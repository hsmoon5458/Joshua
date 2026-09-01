#include "node_generator.h"

#include <glog/logging.h>
#include <limits.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <mutex>
#include <set>
#include <string>
#include <thread>

#include "config/config_utils.h"
#include "utils/status_macros.h"

namespace node_generator {

namespace {
// Common environment variables.
constexpr auto kROS2NodeWrapper = "ros2_node_wrapper.sh";
constexpr auto kROS2 = "ros2";

std::string NodeTypeToString(const ros2::node::NodeType& type) {
  if (type == ros2::node::NODE_INVALID) {
    return "";
  }
  std::string name = ros2::node::NodeType_Name(type);
  std::transform(
      name.begin(), name.end(), name.begin(), [](unsigned char c) { return std::tolower(c); });
  return name;
}

// Resolve current executable absolute path via /proc/self/exe
std::string GetSelfExecutablePath() {
  char buffer[PATH_MAX];
  ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (len == -1) {
    return "";
  }
  buffer[len] = '\0';
  return std::string(buffer);
}

// Try to determine the Bazel runfiles root directory for this process
std::optional<std::filesystem::path> GetRunfilesRoot() {
  const char* env_runfiles = std::getenv("RUNFILES_DIR");
  if (env_runfiles && std::filesystem::exists(env_runfiles)) {
    return std::filesystem::path(env_runfiles);
  }

  // Fallback: <self_exe>.runfiles
  std::string self_exe = GetSelfExecutablePath();
  if (!self_exe.empty()) {
    std::filesystem::path candidate = std::filesystem::path(self_exe + ".runfiles");
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  return std::nullopt;
}

// Try to find the native binary in Bazel's bin tree: <bazel-bin>/ros2/<name>
std::optional<std::filesystem::path> ResolveNativeBinaryInBazelBin(const std::string& exec_name) {
  std::string self_exe = GetSelfExecutablePath();
  if (self_exe.empty()) return std::nullopt;
  std::filesystem::path exe_dir = std::filesystem::path(self_exe).parent_path();
  if (!exe_dir.has_parent_path()) return std::nullopt;
  std::filesystem::path bazel_bin_root = exe_dir.parent_path();
  std::filesystem::path candidate = bazel_bin_root / kROS2 / exec_name;
  if (std::filesystem::exists(candidate)) {
    return candidate;
  }
  return std::nullopt;
}

// Locate the generic wrapper script produced by //ros2:ros2_node_wrapper.sh
std::optional<std::filesystem::path> ResolveRos2WrapperPath() {
  // 1) Next to the current executable (packaged tar layout)
  std::string self_exe = GetSelfExecutablePath();
  if (!self_exe.empty()) {
    std::filesystem::path self_dir = std::filesystem::path(self_exe).parent_path();
    std::filesystem::path candidate = self_dir / kROS2NodeWrapper;
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
    // Also check under a ros2/ subdirectory in the same extracted tree
    std::filesystem::path candidate_sub = self_dir / kROS2 / kROS2NodeWrapper;
    if (std::filesystem::exists(candidate_sub)) {
      return candidate_sub;
    }
  }

  // 2) In this process' runfiles
  if (auto runfiles_root_opt = GetRunfilesRoot()) {
    const std::filesystem::path runfiles_root = *runfiles_root_opt;
    const char* workspace_dirs[] = {"_main", "__main__"};
    for (const char* ws : workspace_dirs) {
      std::filesystem::path ws_dir = runfiles_root / ws;
      std::filesystem::path candidate = ws_dir / kROS2 / kROS2NodeWrapper;
      if (std::filesystem::exists(candidate)) {
        return candidate;
      }
    }
  }

  // 3) In bazel-bin/ros2
  if (!self_exe.empty()) {
    std::filesystem::path exe_dir = std::filesystem::path(self_exe).parent_path();
    if (exe_dir.has_parent_path()) {
      std::filesystem::path bazel_bin_root = exe_dir.parent_path();
      std::filesystem::path candidate = bazel_bin_root / kROS2 / kROS2NodeWrapper;
      if (std::filesystem::exists(candidate)) {
        return candidate;
      }
    }
  }
  return std::nullopt;
}

bool IsExecutablePathValid(const std::filesystem::path& candidate) {
  return std::filesystem::exists(candidate) && access(candidate.c_str(), X_OK) == 0;
}

bool IsExecutableAvailable(const std::string& exec_name) {
  if (auto native_bin = ResolveNativeBinaryInBazelBin(exec_name)) {
    if (IsExecutablePathValid(*native_bin)) {
      return true;
    }
  }

  // Packaged layout: check next to wrapper or under ros2/
  if (auto wrapper = ResolveRos2WrapperPath()) {
    std::filesystem::path wrapper_dir = wrapper->parent_path();
    std::filesystem::path candidate = wrapper_dir / exec_name;
    if (IsExecutablePathValid(candidate)) {
      return true;
    }
    std::filesystem::path candidate_sub = wrapper_dir / kROS2 / exec_name;
    if (IsExecutablePathValid(candidate_sub)) {
      return true;
    }
  }

  // Runfiles layout
  if (auto runfiles_root_opt = GetRunfilesRoot()) {
    const std::filesystem::path runfiles_root = *runfiles_root_opt;
    const char* workspace_dirs[] = {"_main", "__main__"};
    for (const char* ws : workspace_dirs) {
      std::filesystem::path candidate = runfiles_root / ws / kROS2 / exec_name;
      if (IsExecutablePathValid(candidate)) {
        return true;
      }
    }
  }

  return false;
}

void ExtractTopicsFromNode(const ros2::node::Node& node,
                           const uint32_t node_id,
                           std::vector<std::string>& publish_topics,
                           std::vector<std::string>& subscribe_topics) {
  if (node.id() == node_id) {
    for (const auto& pub : node.publishers()) {
      publish_topics.push_back(pub.topic());
    }
    for (const auto& sub : node.subscriptions()) {
      subscribe_topics.push_back(sub.topic());
    }
  }
}
}  // namespace

// Static member initialization
NodeGenerator* NodeGenerator::instance_ = nullptr;

NodeGenerator::NodeGenerator(const std::string& config_path) : shutdown_requested_(false) {
  instance_ = this;

  // Normalize config path so all children receive an absolute path
  // 1) If absolute and exists -> keep
  // 2) If relative and exists from CWD -> make absolute
  // 3) Fallback: try relative to the executable directory
  std::filesystem::path provided_path(config_path);
  std::filesystem::path resolved_path;

  if (provided_path.is_absolute() && std::filesystem::exists(provided_path)) {
    resolved_path = provided_path;
  } else {
    std::filesystem::path abs_from_cwd = std::filesystem::absolute(provided_path);
    if (std::filesystem::exists(abs_from_cwd)) {
      resolved_path = abs_from_cwd;
    } else {
      const std::string self_exe = GetSelfExecutablePath();
      if (!self_exe.empty()) {
        std::filesystem::path exe_dir = std::filesystem::path(self_exe).parent_path();
        std::filesystem::path from_exe_dir = exe_dir / provided_path;
        if (std::filesystem::exists(from_exe_dir)) {
          resolved_path = std::filesystem::absolute(from_exe_dir);
        }
      }
    }
  }

  if (!resolved_path.empty()) {
    config_path_ = resolved_path.lexically_normal().string();
  } else {
    // Fall back to the original value; LoadConfig will report a clear error.
    config_path_ = config_path;
  }
}

NodeGenerator::~NodeGenerator() {
  if (has_nodes()) {
    auto res = Shutdown();
    if (!res.ok()) {
      LOG(ERROR) << "Failed to shutdown nodes";
    }
  }
  instance_ = nullptr;
}

absl::Status NodeGenerator::Initialize() {
  LOG(INFO) << "Initializing NodeGenerator with config: " << config_path_;

  ABSL_ASSIGN_OR_RETURN(config_, config::config_util::LoadConfig(config_path_));

  ABSL_RETURN_IF_ERROR(IdentifyNodeTypes());
  ABSL_RETURN_IF_ERROR(CheckConfigIntegrity());

  SetupSignalHandlers();

  LOG(INFO) << "NodeGenerator initialized successfully";

  return absl::OkStatus();
}

absl::Status NodeGenerator::IdentifyNodeTypes() {
  // Actions
  for (const auto& single_action : config_.robot().actions().single_actions()) {
    const uint32_t node_id = single_action.node().id();
    auto [it, inserted] = identified_nodes_.try_emplace(node_id, single_action.node().node_type());
    if (!inserted && it->second != single_action.node().node_type()) {
      LOG(ERROR) << "Node ID " << node_id << " already exists for node type "
                 << NodeTypeToString(it->second);
      return absl::Status(absl::StatusCode::kInvalidArgument, "Node ID conflict");
    }
  }

  // Perceptions
  for (const auto& single_perception : config_.robot().perceptions().single_perceptions()) {
    const uint32_t node_id = single_perception.node().id();
    auto [it, inserted] =
        identified_nodes_.try_emplace(node_id, single_perception.node().node_type());
    if (!inserted && it->second != single_perception.node().node_type()) {
      LOG(ERROR) << "Node ID " << node_id << " already exists for node type "
                 << NodeTypeToString(it->second);
      return absl::Status(absl::StatusCode::kInvalidArgument, "Node ID conflict");
    }
  }

  // Inference
  for (const auto& single_model : config_.ai().models().single_models()) {
    const uint32_t node_id = single_model.node().id();
    auto [it, inserted] = identified_nodes_.try_emplace(node_id, single_model.node().node_type());
    if (!inserted && it->second != single_model.node().node_type()) {
      LOG(ERROR) << "Node ID " << node_id << " already exists for node type "
                 << NodeTypeToString(it->second);
      return absl::Status(absl::StatusCode::kInvalidArgument, "Node ID conflict");
    }
  }

  // Data store.
  for (const auto& single_data_store : config_.ai().data_stores().single_data_stores()) {
    const uint32_t node_id = single_data_store.node().id();
    auto [it, inserted] =
        identified_nodes_.try_emplace(node_id, single_data_store.node().node_type());
    if (!inserted && it->second != single_data_store.node().node_type()) {
      LOG(ERROR) << "Node ID " << node_id << " already exists for node type "
                 << NodeTypeToString(it->second);
      return absl::Status(absl::StatusCode::kInvalidArgument, "Node ID conflict");
    }
  }

  // Trajectories
  for (const auto& single_trajectory : config_.robot().trajectories().single_trajectories()) {
    const uint32_t node_id = single_trajectory.node().id();
    auto [it, inserted] =
        identified_nodes_.try_emplace(node_id, single_trajectory.node().node_type());
    if (!inserted && it->second != single_trajectory.node().node_type()) {
      LOG(ERROR) << "Node ID " << node_id << " already exists for node type "
                 << NodeTypeToString(it->second);
      return absl::Status(absl::StatusCode::kInvalidArgument, "Node ID conflict");
    }
  }

  return absl::OkStatus();
}

absl::Status NodeGenerator::CheckConfigIntegrity() {
  auto robot_config = config_.robot();

  // A serial port may be opened by exactly one node process. Single-stream
  // sensors (a camera, a lidar) carry their own comm, so their ports are
  // checked directly.
  std::map<std::string, uint32_t> port_to_node_id;
  for (const auto& single_perception : robot_config.perceptions().single_perceptions()) {
    const auto& sensor = single_perception.sensor();
    if (sensor.comm().comm_type() != robot::comm::CommType::SERIAL) {
      continue;
    }
    const std::string& port_name = sensor.comm().serial_config().port();
    if (port_name.empty()) {
      continue;
    }
    const uint32_t node_id = single_perception.node().id();
    auto [it, inserted] = port_to_node_id.try_emplace(port_name, node_id);
    if (!inserted && it->second != node_id) {
      LOG(ERROR) << "Configuration Integrity Failure: Serial port '" << port_name
                 << "' is assigned to multiple node_ids (" << it->second << " and " << node_id
                 << "). This is not allowed as it will cause resource conflicts.";
      return absl::Status(absl::StatusCode::kInvalidArgument, "Serial port conflict");
    }
  }

  // Board synchronization is process-local, so report boards claimed by
  // multiple node processes.
  std::map<std::string, std::set<uint32_t>> board_claimants;
  for (const auto& single_action : robot_config.actions().single_actions()) {
    const auto& board_name = single_action.actuator().board_name();
    if (!board_name.empty()) {
      board_claimants[board_name].insert(single_action.node().id());
    }
  }
  for (const auto& single_perception : robot_config.perceptions().single_perceptions()) {
    const auto& board_name = single_perception.sensor().board_name();
    if (!board_name.empty()) {
      board_claimants[board_name].insert(single_perception.node().id());
    }
  }
  for (const auto& [board_name, node_ids] : board_claimants) {
    if (node_ids.size() > 1) {
      std::string ids;
      for (const uint32_t node_id : node_ids) {
        if (!ids.empty()) {
          ids += ", ";
        }
        ids += std::to_string(node_id);
      }
      LOG(WARNING) << "Board '" << board_name << "' is claimed by node_ids (" << ids
                   << "). Each node runs in its own process, so the board's bus mutex cannot "
                      "serialize them and traffic on its link will interleave.";
    }
  }

  // TODO: Add more check here for the config.

  return absl::OkStatus();
}

absl::Status NodeGenerator::LaunchAllNodes() {
  std::lock_guard<std::mutex> lock(nodes_mutex_);

  for (const auto& [node_id, node_type] : identified_nodes_) {
    const std::string node_name =
        NodeTypeToString(node_type) + std::string("_node_") + std::to_string(node_id);
    pid_t pid = LaunchNode(node_type, node_id, node_name);
    if (pid > 0) {
      std::vector<std::string> publish_topics;
      std::vector<std::string> subscribe_topics;
      ABSL_RETURN_IF_ERROR(GetTopicsForNode(node_id, publish_topics, subscribe_topics));
      launched_nodes_.try_emplace(pid,
                                  NodeInfo{NodeTypeToString(node_type),
                                           node_name,
                                           node_id,
                                           pid,
                                           publish_topics,
                                           subscribe_topics});
    }
  }

  LOG(INFO) << "All " << launched_nodes_.size() << " nodes launched successfully.";
  return !launched_nodes_.empty()
             ? absl::OkStatus()
             : absl::Status(absl::StatusCode::kInvalidArgument, "No nodes launched");
}

pid_t NodeGenerator::LaunchNode(const ros2::node::NodeType& node_type,
                                uint32_t node_id,
                                const std::string& node_name) {
  const std::string node_type_str = NodeTypeToString(node_type);
  if (node_type_str.empty()) {
    LOG(ERROR) << "Invalid node type for node " << node_id;
    return -1;
  }

  // The executable is named for the node type, C++ and Python alike: the
  // hardware-facing nodes are C++, and INFERENCE, DATA_SUBSCRIBER, and
  // TRAJECTORY_PUBLISHER are Python with no C++ counterpart to disambiguate
  // from. Nothing is selected between, so nothing is suffixed.
  const std::string& exec_name = node_type_str;

  if (!IsExecutableAvailable(exec_name)) {
    LOG(ERROR) << "No executable available for node '" << node_name << "' (type '" << node_type_str
               << "').";
    return -1;
  }

  LOG(INFO) << "Launching node '" << node_name << "' (exec '" << exec_name << "').";

  std::string exec_path;
  std::vector<std::string> argv_strings;
  std::vector<std::pair<std::string, std::string>> env_to_set;
  std::vector<std::string> env_to_unset;
  std::string work_dir;

  // 1) Try to execute the native binary with its own runfiles when available (Bazel run).
  if (auto native_bin = ResolveNativeBinaryInBazelBin(exec_name)) {
    exec_path = native_bin->string();
    if (!std::filesystem::exists(exec_path)) {
      LOG(ERROR) << "Native binary not found at '" << exec_path << "' for node '" << exec_name
                 << "'";
      return -1;
    }

    std::filesystem::path bin_path(exec_path);
    std::filesystem::path runfiles_dir =
        bin_path.parent_path() / (bin_path.filename().string() + ".runfiles");

    if (std::filesystem::exists(runfiles_dir)) {
      env_to_set.push_back({"RUNFILES_DIR", runfiles_dir.string()});
      env_to_set.push_back({"TEST_SRCDIR", runfiles_dir.string()});
      env_to_unset = {"RUNFILES_MANIFEST_FILE", "JAVA_RUNFILES"};

      std::filesystem::path wd = runfiles_dir / "_main";
      if (std::filesystem::exists(wd) && std::filesystem::is_directory(wd)) {
        work_dir = wd.string();
      }
    } else {
      // If the child binary does not have its own runfiles (e.g. data dependency),
      // try to inherit the parent's runfiles.
      if (auto parent_runfiles = GetRunfilesRoot()) {
        env_to_set.push_back({"RUNFILES_DIR", parent_runfiles->string()});
        env_to_set.push_back({"TEST_SRCDIR", parent_runfiles->string()});
        env_to_unset = {"RUNFILES_MANIFEST_FILE", "JAVA_RUNFILES"};

        // Try to set work_dir relative to parent runfiles
        std::filesystem::path wd = *parent_runfiles / "_main";
        if (std::filesystem::exists(wd) && std::filesystem::is_directory(wd)) {
          work_dir = wd.string();
        }
      } else {
        env_to_unset = {"RUNFILES_DIR", "TEST_SRCDIR", "RUNFILES_MANIFEST_FILE", "JAVA_RUNFILES"};
      }
    }

    argv_strings = {exec_path, node_name, std::to_string(node_id), config_path_};
  }
  // 2) Try to execute the generic wrapper from packaged tar layout.
  else if (auto wrapper = ResolveRos2WrapperPath()) {
    exec_path = wrapper->string();
    if (!std::filesystem::exists(exec_path)) {
      LOG(ERROR) << "Wrapper script not found at '" << exec_path << "'";
      return -1;
    }
    if (access(exec_path.c_str(), X_OK) != 0) {
      LOG(ERROR) << "Wrapper script not executable: '" << exec_path << "' (chmod +x)";
      return -1;
    }
    argv_strings = {exec_path, exec_name, node_name, std::to_string(node_id), config_path_};
  } else {
    LOG(ERROR) << "Could not locate ROS2 wrapper, native binary, or launch for node type '"
               << exec_name << "'";
    return -1;
  }

  // Prepare argv pointers
  std::vector<char*> argv_ptrs;
  argv_ptrs.reserve(argv_strings.size() + 1);
  for (const auto& s : argv_strings) {
    argv_ptrs.push_back(const_cast<char*>(s.c_str()));
  }
  argv_ptrs.push_back(nullptr);

  pid_t pid = fork();

  if (pid == 0) {
    // Put child in its own process group so we can signal the whole group (including grandchildren)
    if (setpgid(0, 0) != 0) {
      // Failed to set process group. This is not critical.
    }

    for (const auto& [k, v] : env_to_set) {
      setenv(k.c_str(), v.c_str(), 1);
    }
    for (const auto& k : env_to_unset) {
      unsetenv(k.c_str());
    }

    if (!work_dir.empty()) {
      if (chdir(work_dir.c_str()) != 0) {
        // Failed to chdir. Proceeding anyway as it might still work.
      }
    }

    execv(exec_path.c_str(), argv_ptrs.data());

    const char* msg = "Failed to execute node process\n";
    write(STDERR_FILENO, msg, strlen(msg));
    _exit(1);
  } else if (pid > 0) {
    LOG(INFO) << "Launched " << node_name << " with PID: " << pid;
    return pid;
  } else {
    LOG(ERROR) << "Failed to fork process for " << node_name << ": " << strerror(errno);
    return -1;
  }
}

absl::Status NodeGenerator::MonitorNodes() {
  LOG(INFO) << "Starting node monitoring...";

  while (!shutdown_requested_) {
    MonitorChildProcesses();

    bool empty;
    {
      std::lock_guard<std::mutex> lock(nodes_mutex_);
      empty = launched_nodes_.empty();
    }

    if (empty) {
      LOG(WARNING) << "All nodes have terminated. Exiting...";
      CleanupAndExit(1);
    }

    // Avoid busy-waiting.
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  LOG(INFO) << "Shutdown requested, cleaning up...";
  CleanupAndExit(0);
  return absl::OkStatus();
}

void NodeGenerator::MonitorChildProcesses() {
  int status;
  while (true) {
    // waitpid(-1) with WNOHANG checks for ANY child process that has exited.
    // It returns 0 if there are children but none have exited.
    // It returns -1 if there are no children (errno=ECHILD) or on error.
    // It returns > 0 (the PID) if a child exited.
    pid_t pid = waitpid(-1, &status, WNOHANG);

    if (pid == 0) {
      // No children have exited this tick.
      break;
    }

    if (pid == -1) {
      if (errno != ECHILD) {
        LOG(ERROR) << "Error waiting for child processes: " << strerror(errno);
      }
      break;
    }

    // A child (pid) has exited.
    std::lock_guard<std::mutex> lock(nodes_mutex_);
    auto it = launched_nodes_.find(pid);
    if (it != launched_nodes_.end()) {
      if (WIFEXITED(status)) {
        LOG(ERROR) << "Node " << it->second.node_name << " (PID " << pid
                   << ") exited with status: " << WEXITSTATUS(status);
      } else if (WIFSIGNALED(status)) {
        LOG(ERROR) << "Node " << it->second.node_name << " (PID " << pid
                   << ") terminated by signal: " << WTERMSIG(status);
      }
      launched_nodes_.erase(it);
    } else {
      // This might happen if a child double-forked or is not tracked.
      // Just log it.
      LOG(WARNING) << "Reaped unknown child process PID: " << pid;
    }
  }
}

void NodeGenerator::KillAllNodes(int signal) {
  for (const auto& [pid, node] : launched_nodes_) {
    if (pid > 0) {
      kill(-pid, signal);
    }
  }
}

bool NodeGenerator::WaitForNodesToExit(int timeout_ms) {
  int waited = 0;
  const int poll_ms = 100;
  while (waited < timeout_ms) {
    // Reap any zombies
    while (true) {
      int status;
      pid_t pid = waitpid(-1, &status, WNOHANG);
      if (pid <= 0) break;  // 0=running, -1=error/none

      auto it = launched_nodes_.find(pid);
      if (it != launched_nodes_.end()) {
        launched_nodes_.erase(it);
      }
    }

    if (launched_nodes_.empty()) return true;

    std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
    waited += poll_ms;
  }
  return launched_nodes_.empty();
}

absl::Status NodeGenerator::Shutdown(const int max_wait_ms) {
  LOG(INFO) << "Shutting down all nodes...";

  std::lock_guard<std::mutex> lock(nodes_mutex_);

  const int int_wait_ms = static_cast<int>(max_wait_ms);
  const int term_wait_ms = max_wait_ms - int_wait_ms;

  // 1. SIGINT (Graceful shutdown)
  KillAllNodes(SIGINT);
  if (WaitForNodesToExit(int_wait_ms)) {
    LOG(INFO) << "All processes terminated successfully (SIGINT).";
    return absl::OkStatus();
  }

  // 2. SIGTERM (Forceful request)
  LOG(WARNING) << "Nodes still running. Sending SIGTERM...";
  KillAllNodes(SIGTERM);
  if (WaitForNodesToExit(term_wait_ms)) {
    LOG(INFO) << "All processes terminated successfully (SIGTERM).";
    return absl::OkStatus();
  }

  // 3. SIGKILL (Last resort)
  LOG(ERROR) << "Nodes did not exit. Sending SIGKILL...";
  KillAllNodes(SIGKILL);
  WaitForNodesToExit(1000);  // Give kernel a moment to clean up

  if (!launched_nodes_.empty()) {
    LOG(ERROR) << "Failed to kill " << launched_nodes_.size() << " nodes (Zombies?)";
    launched_nodes_.clear();  // Clear map anyway since we can't do anything else
  } else {
    LOG(INFO) << "All processes terminated (SIGKILL).";
  }

  return absl::OkStatus();
}

absl::Status NodeGenerator::GetLaunchedNodes(std::vector<NodeInfo>& nodes) {
  std::lock_guard<std::mutex> lock(nodes_mutex_);
  nodes.clear();
  nodes.reserve(launched_nodes_.size());
  for (const auto& [pid, info] : launched_nodes_) {
    nodes.push_back(info);
  }
  return absl::OkStatus();
}

void NodeGenerator::SetupSignalHandlers() {
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = SignalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;

  if (sigaction(SIGINT, &sa, nullptr) == -1) {
    LOG(ERROR) << "Failed to set SIGINT handler: " << strerror(errno);
  }
  if (sigaction(SIGTERM, &sa, nullptr) == -1) {
    LOG(ERROR) << "Failed to set SIGTERM handler: " << strerror(errno);
  }
}

void NodeGenerator::CleanupAndExit(int exit_code) {
  auto res = Shutdown();
  if (!res.ok()) {
    LOG(ERROR) << "Failed to shutdown nodes";
  }
  exit(exit_code);
}

void NodeGenerator::SignalHandler(int sig) {
  // Use write() instead of LOG() because it is async-signal-safe.
  // LOG() uses mutexes and malloc, which can deadlock if the signal interrupted LOG().
  const char* msg = (sig == SIGINT) ? "\nReceived SIGINT, shutting down...\n"
                                    : "\nReceived SIGTERM, shutting down...\n";
  write(STDERR_FILENO, msg, strlen(msg));

  if (instance_) {
    instance_->shutdown_requested_ = true;
  }
}

absl::Status NodeGenerator::GetTopicsForNode(const uint32_t node_id,
                                             std::vector<std::string>& publish_topics,
                                             std::vector<std::string>& subscribe_topics) {
  if (identified_nodes_.count(node_id) == 0) {
    LOG(WARNING) << "Node " << node_id << " not found in the config.";
    return absl::Status(absl::StatusCode::kInvalidArgument, "Node not found in the config.");
  }

  for (const auto& single_perception : config_.robot().perceptions().single_perceptions()) {
    ExtractTopicsFromNode(single_perception.node(), node_id, publish_topics, subscribe_topics);
  }

  for (const auto& single_action : config_.robot().actions().single_actions()) {
    ExtractTopicsFromNode(single_action.node(), node_id, publish_topics, subscribe_topics);
  }

  for (const auto& single_model : config_.ai().models().single_models()) {
    ExtractTopicsFromNode(single_model.node(), node_id, publish_topics, subscribe_topics);
  }

  for (const auto& single_data_store : config_.ai().data_stores().single_data_stores()) {
    ExtractTopicsFromNode(single_data_store.node(), node_id, publish_topics, subscribe_topics);
  }

  for (const auto& single_trajectory : config_.robot().trajectories().single_trajectories()) {
    ExtractTopicsFromNode(single_trajectory.node(), node_id, publish_topics, subscribe_topics);
  }

  return absl::OkStatus();
}
}  // namespace node_generator

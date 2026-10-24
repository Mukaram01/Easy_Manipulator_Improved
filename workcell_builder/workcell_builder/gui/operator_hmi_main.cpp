// Workcell Operator HMI entry point.
//
//   workcell_operator_hmi --request <file> --managed   (launched by Workcell Builder)
//   workcell_operator_hmi --scene <scene_dir>           (standalone, saved revision)

#include <QApplication>
#include <QCommandLineParser>
#include <QSocketNotifier>

#include <csignal>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include "operator_hmi_window.hpp"
#include "workcell_builder_ui_utils.hpp"

namespace
{
int g_signal_fd[2] = {-1, -1};

void on_terminate(int signal_number)
{
  static_cast<void>(signal_number);
  const char byte = 1;
  [[maybe_unused]] const auto written = ::write(g_signal_fd[0], &byte, 1);
}
}  // namespace

int main(int argc, char ** argv)
{
  // The operator HMI is dark by default; WORKCELL_STUDIO_APPEARANCE still wins.
  if (qEnvironmentVariableIsEmpty("WORKCELL_STUDIO_APPEARANCE")) qputenv("WORKCELL_STUDIO_APPEARANCE", "dark");
  QApplication app(argc, argv);
  QApplication::setApplicationName("workcell_operator_hmi");
  QApplication::setOrganizationName("WorkcellStudio");

  QCommandLineParser parser;
  parser.setApplicationDescription("Workcell Operator HMI (preview, no robot motion)");
  parser.addHelpOption();
  QCommandLineOption request_option("request", "Workcell Builder preview request file.", "file");
  QCommandLineOption scene_option("scene", "Open the saved revision of this scene directory.", "dir");
  QCommandLineOption scripts_option("scripts-dir", "Directory holding the Workcell Studio helper scripts.", "dir");
  QCommandLineOption status_option("status-out", "Write machine-readable HMI status here (acceptance runs).", "file");
  QCommandLineOption managed_option("managed", "Exit when the launching Workcell Builder exits.");
  QCommandLineOption commands_option("acceptance-commands", "Test only: command file driving the HMI controls.", "file");
  QCommandLineOption capture_option("capture-dir", "Test only: directory for requested window captures.", "dir");
  parser.addOptions({request_option, scene_option, scripts_option, status_option, managed_option,
                     commands_option, capture_option});
  parser.process(app);

  if (parser.isSet(managed_option)) {
    // A preview window must not outlive the Builder that owns it.
    ::prctl(PR_SET_PDEATHSIG, SIGTERM);
    if (::getppid() == 1) return 0;
  }
  // SIGTERM/SIGINT end the event loop normally so running checks are stopped.
  if (::socketpair(AF_UNIX, SOCK_STREAM, 0, g_signal_fd) == 0) {
    auto * notifier = new QSocketNotifier(g_signal_fd[1], QSocketNotifier::Read, &app);
    QObject::connect(notifier, &QSocketNotifier::activated, &app, [&app]() {
        char byte = 0;
        [[maybe_unused]] const auto bytes = ::read(g_signal_fd[1], &byte, 1);
        app.closeAllWindows();
        app.quit();
      });
    struct sigaction action{};
    action.sa_handler = on_terminate;
    sigemptyset(&action.sa_mask);
    action.sa_flags = SA_RESTART;
    ::sigaction(SIGTERM, &action, nullptr);
    ::sigaction(SIGINT, &action, nullptr);
  }

  if (!parser.isSet(request_option) && !parser.isSet(scene_option)) {
    parser.showHelp(2);
  }
  workcell_builder::applyStudioApplicationAppearance(&app);
  workcell_builder::OperatorHmiOptions options;
  options.request_path = parser.value(request_option);
  options.scene_dir = parser.value(scene_option);
  options.scripts_dir = parser.value(scripts_option);
  options.status_out = parser.value(status_option);
  options.acceptance_commands = parser.value(commands_option);
  options.capture_dir = parser.value(capture_option);
  workcell_builder::OperatorHmiWindow window(options);
  window.show();
  window.start();
  return app.exec();
}

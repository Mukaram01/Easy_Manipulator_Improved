#include "gui/transform_clipboard_utils.h"
#include <QRegularExpression>
#include <array>
#include <cmath>

namespace workcell_builder {
bool parse_transform_clipboard_text(const QString & text, double * x, double * y, double * z, double * r, double * p, double * yaw, QString * error)
{
  const QRegularExpression re(
    R"(^\s*x\s*=\s*([^\s]+)\s+y\s*=\s*([^\s]+)\s+z\s*=\s*([^\s]+)\s+r\s*=\s*([^\s]+)\s+p\s*=\s*([^\s]+)\s+yaw\s*=\s*([^\s]+)\s*$)",
    QRegularExpression::CaseInsensitiveOption);
  const auto m = re.match(text);
  if (!m.hasMatch()) {
    if (error) *error = "Expected format: x=<num> y=<num> z=<num> r=<num> p=<num> yaw=<num>";
    return false;
  }
  bool ok = false;
  std::array<double *, 6> out{ x, y, z, r, p, yaw };
  for (int i = 0; i < 6; ++i) {
    const double v = m.captured(i + 1).toDouble(&ok);
    if (!ok || !std::isfinite(v)) {
      if (error) *error = QString("Invalid numeric value for token #%1").arg(i + 1);
      return false;
    }
    if (out[i]) *out[i] = v;
  }
  return true;
}
}

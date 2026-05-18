#ifndef WORKCELL_BUILDER__GUI__TRANSFORM_CLIPBOARD_UTILS_H_
#define WORKCELL_BUILDER__GUI__TRANSFORM_CLIPBOARD_UTILS_H_
#include <QString>
namespace workcell_builder {
bool parse_transform_clipboard_text(const QString & text, double * x, double * y, double * z, double * r, double * p, double * yaw, QString * error = nullptr);
}
#endif

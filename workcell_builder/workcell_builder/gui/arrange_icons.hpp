#pragma once

// Vector line icons for the arrangement toolbar (painted at 1x and 2x; the
// reference part in the accent colour; a muted Disabled rendering).

#include <QIcon>
#include <QString>

#include <cmath>

namespace workcell_builder
{

// align_left|right|front|back|top|bottom, centre_x|y|z, centre_xy, centre_ref,
// place_top, dist_x|y|z, gap, move, snap_grid, mount, detach, flush, coincident,
// concentric, group.
QIcon arrangeIcon(const QString & name);

}  // namespace workcell_builder

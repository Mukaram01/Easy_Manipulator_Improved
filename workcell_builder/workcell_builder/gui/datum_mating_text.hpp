#pragma once

// Beginner-facing text for the fixed mounting / datum-mate relation kept in an
// asset's mounting metadata. scripts/asset_mounting.py owns the transform math
// and validation; this header only reads the relation to describe it.

#include <QString>
#include <QStringList>
#include <QtMath>

#include <cmath>

#include <yaml-cpp/yaml.h>

namespace workcell_builder
{

struct MountRelation
{
  bool mounted{false};       // parent_object names another asset
  bool datum_mate{false};    // explicit `mate` mapping (child datum / alignment)
  QString parent_id;
  QString parent_datum{QStringLiteral("origin")};
  QString child_datum{QStringLiteral("origin")};
  QString alignment{QStringLiteral("coincident")};
  double offset[6]{0., 0., 0., 0., 0., 0.};  // metres, radians (parent datum frame)
};

inline MountRelation parse_mount_relation(const QString & mounting_yaml)
{
  MountRelation relation;
  YAML::Node node;
  try { node = YAML::Load(mounting_yaml.toStdString()); } catch (const YAML::Exception &) { return relation; }
  if (!node || !node.IsMap() || !node["parent_object"]) return relation;
  const auto parent = QString::fromStdString(node["parent_object"].as<std::string>(""));
  if (parent.isEmpty() || parent == QStringLiteral("world")) return relation;
  relation.mounted = true;
  relation.parent_id = parent;
  relation.parent_datum = QString::fromStdString(node["parent_link"].as<std::string>("origin"));
  if (node["mate"] && node["mate"].IsMap()) {
    relation.datum_mate = true;
    relation.child_datum = QString::fromStdString(node["mate"]["child_datum"].as<std::string>("origin"));
    relation.alignment = QString::fromStdString(node["mate"]["alignment"].as<std::string>("coincident"));
  }
  if (node["origin"] && node["origin"].IsMap()) {
    const char * keys[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
    for (int k = 0; k < 6; ++k) relation.offset[k] = node["origin"][keys[k]].as<double>(0.);
  }
  return relation;
}

// Same relation regardless of YAML style (the resolver returns flow JSON, a
// reopened scene block YAML); numbers compare exactly up to 1e-12.
inline bool same_yaml_value(const YAML::Node & a, const YAML::Node & b)
{
  const bool a_empty = !a || a.IsNull(), b_empty = !b || b.IsNull();
  if (a_empty || b_empty) return a_empty == b_empty;
  if (a.Type() != b.Type()) return false;
  if (a.IsMap()) {
    if (a.size() != b.size()) return false;
    for (const auto & entry : a) if (!same_yaml_value(entry.second, b[entry.first.as<std::string>()])) return false;
    return true;
  }
  if (a.IsSequence()) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) if (!same_yaml_value(a[i], b[i])) return false;
    return true;
  }
  try { return std::abs(a.as<double>() - b.as<double>()) <= 1e-12; }
  catch (const YAML::Exception &) { return a.as<std::string>() == b.as<std::string>(); }
}

inline bool same_mounting_metadata(const QString & a, const QString & b)
{
  try {
    return same_yaml_value(YAML::Load(a.toStdString()), YAML::Load(b.toStdString()));
  } catch (const YAML::Exception &) {
    return a == b;
  }
}

inline QString alignment_text(const QString & alignment)
{
  return alignment == QStringLiteral("flush") ? QStringLiteral("flush (face to face)") : QStringLiteral("coincident");
}

// "bottom · plane — Bottom face (normal points down)"
inline QString datum_choice_text(const QString & name, const QString & type, const QString & label = QString())
{
  QString text = QStringLiteral("%1 · %2").arg(name, type.isEmpty() ? QStringLiteral("point") : type);
  if (!label.trimmed().isEmpty()) text += QStringLiteral(" — ") + label.trimmed();
  return text;
}

inline QString mount_offset_text(const MountRelation & relation)
{
  // Rounded display never shows "-0.000".
  const auto metres = [](double v) { return QString::number(std::abs(v) < 5e-4 ? 0. : v, 'f', 3); };
  const auto degrees = [](double v) { const double d = qRadiansToDegrees(v); return QString::number(std::abs(d) < 0.05 ? 0. : d, 'f', 1); };
  return QStringLiteral("Offset X %1 · Y %2 · Z %3 m\nRoll %4° · pitch %5° · yaw %6°")
    .arg(metres(relation.offset[0]), metres(relation.offset[1]), metres(relation.offset[2]),
         degrees(relation.offset[3]), degrees(relation.offset[4]), degrees(relation.offset[5]));
}

// One line for the hierarchy row: "bottom flush on surface".
inline QString mount_relation_short(const MountRelation & relation)
{
  if (!relation.mounted) return QString();
  if (!relation.datum_mate) return QStringLiteral("mounted at %1").arg(relation.parent_datum);
  return QStringLiteral("%1 %2 %3").arg(relation.child_datum,
    relation.alignment == QStringLiteral("flush") ? QStringLiteral("flush on") : QStringLiteral("coincident with"),
    relation.parent_datum);
}

// Selection context: what the asset is attached to and how.
inline QString mount_relation_summary(const MountRelation & relation, const QString & parent_name)
{
  if (!relation.mounted) return QStringLiteral("Not mounted · placed in world coordinates");
  const QString parent = parent_name.trimmed().isEmpty() ? relation.parent_id : parent_name.trimmed();
  if (!relation.datum_mate) {
    return QStringLiteral("Mounted to %1 · %2\n%3").arg(parent, relation.parent_datum, mount_offset_text(relation));
  }
  return QStringLiteral("Mated: %1 %2 to %3 · %4\n%5").arg(relation.child_datum, alignment_text(relation.alignment),
    parent, relation.parent_datum, mount_offset_text(relation));
}

}  // namespace workcell_builder

#pragma once

#include <QDateTime>
#include <QHash>
#include <QImage>
#include <QList>
#include <QObject>
#include <QSize>
#include <QString>
#include <QSet>

class AssetThumbnailService : public QObject
{
  Q_OBJECT
public:
  enum class Status { Missing, Queued, Rendering, Ready, Failed };
  Q_ENUM(Status)

  struct Request {
    QString asset_id;
    QString resolved_mesh_path;
    double mesh_scale{1.0};
    QSize image_size{256, 192};
  };
  struct Result {
    QString asset_id;
    Status status{Status::Missing};
    QImage image;
    QString source_fingerprint;
    QDateTime generated_at;
    QString error;
    QString cache_key;
  };

  explicit AssetThumbnailService(QObject * parent = nullptr, const QString & cache_root = QString());
  ~AssetThumbnailService() override;
  Result request(const Request & request);
  Result result(const QString & asset_id) const;
  QString cache_root() const;

  static QString source_fingerprint(const Request & request);
  static QString cache_key(const Request & request);
  static QString thumbnail_identity(const QString & catalog_asset_id);
  static Result render_now(const Request & request);

signals:
  void thumbnailChanged(const QString & asset_id);

private:
  void start_next();
  QString cache_path(const QString & key) const;

  QString cache_root_;
  QHash<QString, Result> results_;
  QList<Request> queue_;
  QSet<QString> queued_keys_;
  bool worker_active_{false};
};

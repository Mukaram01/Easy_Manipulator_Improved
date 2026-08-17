#include "gui/asset_thumbnail_service.h"

#include <QCryptographicHash>
#include <QDir>
#include <QFileInfo>
#include <QFutureWatcher>
#include <QDebug>
#include <QPainter>
#include <QPolygonF>
#include <QStandardPaths>
#include <QtConcurrent>
#include <QVector3D>
#include <algorithm>
#include <cmath>

#ifdef WORKCELL_BUILDER_HAS_ASSIMP
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>
#endif

namespace {
struct Triangle { QVector3D a, b, c, normal; double depth{0.0}; };
QString status_error(const QString & text) { return text.isEmpty() ? QStringLiteral("unknown rendering error") : text; }
}

AssetThumbnailService::AssetThumbnailService(QObject * parent, const QString & cache_root)
: QObject(parent), cache_root_(cache_root)
{
  if (cache_root_.isEmpty()) {
    cache_root_ = QDir(QStandardPaths::writableLocation(QStandardPaths::CacheLocation))
      .filePath(QStringLiteral("asset-thumbnails/v1"));
  }
  QDir().mkpath(cache_root_);
}
AssetThumbnailService::~AssetThumbnailService() = default;
QString AssetThumbnailService::cache_root() const { return cache_root_; }
QString AssetThumbnailService::thumbnail_identity(const QString & catalog_asset_id) { return catalog_asset_id.trimmed(); }

QString AssetThumbnailService::source_fingerprint(const Request & r)
{
  QFileInfo f(r.resolved_mesh_path);
  const QString canonical = f.canonicalFilePath().isEmpty() ? f.absoluteFilePath() : f.canonicalFilePath();
  return QStringLiteral("%1|%2|%3|scale=%4|size=%5x%6")
    .arg(canonical).arg(f.exists() ? f.size() : -1)
    .arg(f.exists() ? f.lastModified().toMSecsSinceEpoch() : -1)
    .arg(r.mesh_scale, 0, 'g', 17).arg(r.image_size.width()).arg(r.image_size.height());
}
QString AssetThumbnailService::cache_key(const Request & r)
{
  const QByteArray material = (thumbnail_identity(r.asset_id) + "|" + source_fingerprint(r)).toUtf8();
  return QString::fromLatin1(QCryptographicHash::hash(material, QCryptographicHash::Sha256).toHex());
}
QString AssetThumbnailService::cache_path(const QString & key) const { return QDir(cache_root_).filePath(key + ".png"); }

AssetThumbnailService::Result AssetThumbnailService::result(const QString & id) const
{
  return results_.value(thumbnail_identity(id), Result{id, Status::Missing});
}

AssetThumbnailService::Result AssetThumbnailService::request(const Request & request)
{
  const QString id = thumbnail_identity(request.asset_id);
  const QString key = cache_key(request);
  auto current = results_.value(id, Result{id, Status::Missing});
  if (current.cache_key == key && (current.status == Status::Ready || current.status == Status::Queued || current.status == Status::Rendering)) return current;
  QImage cached(cache_path(key));
  if (!cached.isNull()) {
    Result hit{id, Status::Ready, cached, source_fingerprint(request), QFileInfo(cache_path(key)).lastModified(), {}, key};
    results_[id] = hit;
    qInfo("Asset thumbnail cache hit: asset=%s key=%s", qPrintable(id), qPrintable(key));
    return hit;
  }
  Result queued{id, Status::Queued, {}, source_fingerprint(request), {}, {}, key};
  results_[id] = queued;
  if (!queued_keys_.contains(key)) { queue_.append(request); queued_keys_.insert(key); }
  emit thumbnailChanged(id);
  start_next();
  return queued;
}

void AssetThumbnailService::start_next()
{
  if (worker_active_ || queue_.isEmpty()) return;
  worker_active_ = true;
  const Request request = queue_.takeFirst();
  const QString id = thumbnail_identity(request.asset_id);
  const QString key = cache_key(request);
  results_[id].status = Status::Rendering;
  emit thumbnailChanged(id);
  auto * watcher = new QFutureWatcher<Result>(this);
  connect(watcher, &QFutureWatcher<Result>::finished, this, [this, watcher, request, id, key]() {
    Result rendered = watcher->result();
    watcher->deleteLater();
    queued_keys_.remove(key);
    // A newer source fingerprint supersedes an in-flight result.
    if (results_.value(id).cache_key == key) {
      rendered.cache_key = key;
      rendered.source_fingerprint = source_fingerprint(request);
      rendered.generated_at = QDateTime::currentDateTimeUtc();
      if (rendered.status == Status::Ready) {
        QDir().mkpath(cache_root_);
        if (!rendered.image.save(cache_path(key), "PNG")) {
          rendered.status = Status::Failed;
          rendered.error = QStringLiteral("could not write thumbnail cache %1").arg(cache_path(key));
        }
      }
      results_[id] = rendered;
      if (rendered.status == Status::Ready)
        qInfo("Asset thumbnail generated: asset=%s size=%dx%d", qPrintable(id), rendered.image.width(), rendered.image.height());
      else qWarning("Asset thumbnail failed: asset=%s reason=%s", qPrintable(id), qPrintable(rendered.error));
      emit thumbnailChanged(id);
    }
    worker_active_ = false;
    start_next();
  });
  watcher->setFuture(QtConcurrent::run([request]() { return render_now(request); }));
}

AssetThumbnailService::Result AssetThumbnailService::render_now(const Request & request)
{
  Result out; out.asset_id = thumbnail_identity(request.asset_id);
  const QSize size = request.image_size.isValid() ? request.image_size : QSize(256, 192);
  QFileInfo file(request.resolved_mesh_path);
  if (!file.isFile()) { out.status = Status::Failed; out.error = QStringLiteral("mesh file is unavailable: %1").arg(request.resolved_mesh_path); return out; }
  QVector<Triangle> triangles;
#ifdef WORKCELL_BUILDER_HAS_ASSIMP
  Assimp::Importer importer;
  const aiScene * scene = importer.ReadFile(file.absoluteFilePath().toStdString(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices | aiProcess_GenSmoothNormals | aiProcess_PreTransformVertices);
  if (!scene) { out.status = Status::Failed; out.error = status_error(QString::fromUtf8(importer.GetErrorString())); return out; }
  for (unsigned mi = 0; mi < scene->mNumMeshes; ++mi) {
    const aiMesh * mesh = scene->mMeshes[mi];
    for (unsigned fi = 0; fi < mesh->mNumFaces; ++fi) {
      const aiFace & face = mesh->mFaces[fi]; if (face.mNumIndices != 3) continue;
      Triangle t;
      QVector3D * v[3] = {&t.a, &t.b, &t.c};
      for (int i=0;i<3;++i) { const auto & p=mesh->mVertices[face.mIndices[i]]; *v[i]=QVector3D(p.x,p.y,p.z)*request.mesh_scale; }
      t.normal = QVector3D::normal(t.b-t.a, t.c-t.a); triangles.append(t);
    }
  }
#else
  out.status = Status::Failed; out.error = QStringLiteral("thumbnail rendering requires the existing Assimp mesh-loader dependency"); return out;
#endif
  if (triangles.isEmpty()) { out.status=Status::Failed; out.error=QStringLiteral("mesh contains no physical triangles"); return out; }
  QVector3D minv=triangles[0].a,maxv=minv;
  for (const auto & t:triangles) for (const auto & p:{t.a,t.b,t.c}) { minv.setX(std::min(minv.x(),p.x())); minv.setY(std::min(minv.y(),p.y())); minv.setZ(std::min(minv.z(),p.z())); maxv.setX(std::max(maxv.x(),p.x())); maxv.setY(std::max(maxv.y(),p.y())); maxv.setZ(std::max(maxv.z(),p.z())); }
  const QVector3D center=(minv+maxv)*0.5f;
  const QVector3D forward=QVector3D(-1,-1,-0.65f).normalized(), right=QVector3D::crossProduct(forward,QVector3D(0,0,1)).normalized(), up=QVector3D::crossProduct(right,forward).normalized();
  float minx=1e30f,maxx=-1e30f,miny=1e30f,maxy=-1e30f;
  auto project=[&](const QVector3D&p){QVector3D q=p-center;return QPointF(QVector3D::dotProduct(q,right),QVector3D::dotProduct(q,up));};
  for (const auto&t:triangles) for(const auto&p:{t.a,t.b,t.c}) {auto q=project(p);minx=std::min(minx,float(q.x()));maxx=std::max(maxx,float(q.x()));miny=std::min(miny,float(q.y()));maxy=std::max(maxy,float(q.y()));}
  const float extent=std::max({maxx-minx,maxy-miny,1e-9f}); const float scale=0.78f*std::min(size.width(),size.height())/extent;
  for(auto&t:triangles)t.depth=(QVector3D::dotProduct(t.a-center,forward)+QVector3D::dotProduct(t.b-center,forward)+QVector3D::dotProduct(t.c-center,forward))/3.0;
  std::sort(triangles.begin(),triangles.end(),[](const Triangle&a,const Triangle&b){return a.depth>b.depth;});
  out.image=QImage(size,QImage::Format_ARGB32_Premultiplied); out.image.fill(QColor("#eef2f5")); QPainter painter(&out.image); painter.setRenderHint(QPainter::Antialiasing);
  const QPointF image_center(size.width()/2.0,size.height()/2.0); const QVector3D light=QVector3D(0.3f,-0.5f,1).normalized();
  for(const auto&t:triangles){QPolygonF poly;for(const auto&p:{t.a,t.b,t.c}){auto q=project(p);poly<<image_center+QPointF(q.x()*scale,-q.y()*scale);} const float lum=0.55f+0.35f*std::abs(QVector3D::dotProduct(t.normal,light)); QColor c; c.setRgbF(0.20f*lum,0.48f*lum,0.68f*lum);painter.setPen(QPen(c.darker(125),0.45));painter.setBrush(c);painter.drawPolygon(poly);}
  painter.end(); out.status=Status::Ready; return out;
}

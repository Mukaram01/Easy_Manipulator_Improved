#include <gtest/gtest.h>
#include <QDir>
#include <QFile>
#include <QTemporaryDir>
#include "gui/asset_thumbnail_service.h"

namespace {
void write_ascii_cube(const QString & path, double side)
{
  QFile f(path); ASSERT_TRUE(f.open(QIODevice::WriteOnly | QIODevice::Text));
  const double s=side; const int faces[12][3]={{0,1,2},{0,2,3},{4,6,5},{4,7,6},{0,4,5},{0,5,1},{1,5,6},{1,6,2},{2,6,7},{2,7,3},{3,7,4},{3,4,0}};
  const double v[8][3]={{0,0,0},{s,0,0},{s,s,0},{0,s,0},{0,0,s},{s,0,s},{s,s,s},{0,s,s}};
  QByteArray text("solid cube\n");
  for(auto & face:faces){text += "facet normal 0 0 1\n outer loop\n";for(int i:face)text += QByteArray("vertex ")+QByteArray::number(v[i][0])+" "+QByteArray::number(v[i][1])+" "+QByteArray::number(v[i][2])+"\n";text += "endloop\nendfacet\n";} text += "endsolid cube\n"; f.write(text);
}
}

TEST(AssetThumbnailService, StableCacheKeyAndCatalogIdentity)
{
  QTemporaryDir dir; const QString mesh=dir.filePath("part.stl"); write_ascii_cube(mesh,1000);
  AssetThumbnailService::Request a{"2068_001_24",mesh,0.001,{256,192}}, b=a;
  const QString original_key=AssetThumbnailService::cache_key(a);
  EXPECT_EQ(original_key,AssetThumbnailService::cache_key(b));
  EXPECT_EQ(AssetThumbnailService::thumbnail_identity("2068_001_24"),AssetThumbnailService::thumbnail_identity("2068_001_24"));
  QFile f(mesh); ASSERT_TRUE(f.open(QIODevice::Append)); f.write("\n"); f.close();
  EXPECT_NE(original_key,AssetThumbnailService::cache_key(b));
}

TEST(AssetThumbnailService, FailureDoesNotChangeAssetValidity)
{
  auto result=AssetThumbnailService::render_now({"valid_catalog_asset","/missing.stl",1.0,{256,192}});
  EXPECT_EQ(result.status,AssetThumbnailService::Status::Failed); EXPECT_FALSE(result.error.isEmpty());
  EXPECT_EQ(result.asset_id,"valid_catalog_asset");
}

TEST(AssetThumbnailService, CacheHitAvoidsRendering)
{
  QTemporaryDir source_dir, cache_dir;
  const QString mesh=source_dir.filePath("part.stl"); write_ascii_cube(mesh,1);
  AssetThumbnailService::Request request{"catalog_part",mesh,1.0,{64,48}};
  QImage stored(request.image_size,QImage::Format_ARGB32); stored.fill(Qt::red);
  ASSERT_TRUE(stored.save(cache_dir.filePath(AssetThumbnailService::cache_key(request)+".png")));
  AssetThumbnailService service(nullptr,cache_dir.path());
  const auto hit=service.request(request);
  EXPECT_EQ(hit.status,AssetThumbnailService::Status::Ready);
  EXPECT_EQ(hit.image.pixelColor(2,2),QColor(Qt::red));
}

#ifdef WORKCELL_BUILDER_HAS_ASSIMP
TEST(AssetThumbnailService, RendersStlWithSingleScaleAndCameraFit)
{
  QTemporaryDir dir; const QString mesh=dir.filePath("millimetre_cube.stl"); write_ascii_cube(mesh,1000);
  const auto mm=AssetThumbnailService::render_now({"cube",mesh,0.001,{256,192}});
  const auto metres=AssetThumbnailService::render_now({"cube_large",mesh,1.0,{256,192}});
  ASSERT_EQ(mm.status,AssetThumbnailService::Status::Ready); ASSERT_FALSE(mm.image.isNull());
  ASSERT_EQ(metres.status,AssetThumbnailService::Status::Ready);
  auto occupation=[](const QImage&i){int n=0;for(int y=0;y<i.height();++y)for(int x=0;x<i.width();++x)if(i.pixelColor(x,y)!=QColor("#eef2f5"))++n;return n;};
  EXPECT_GT(occupation(mm.image),mm.image.width()*mm.image.height()/12);
  EXPECT_GT(occupation(metres.image),metres.image.width()*metres.image.height()/12);
  EXPECT_NEAR(double(occupation(mm.image))/occupation(metres.image),1.0,0.05);
}
#endif

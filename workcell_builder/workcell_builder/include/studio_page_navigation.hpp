#pragma once

#include <QListWidget>
#include <QStackedWidget>

namespace workcell_builder
{

inline bool synchronize_studio_page(
  QListWidget * navigation, QStackedWidget * pages, int requested_page)
{
  if (!navigation || !pages || requested_page < 0 ||
    requested_page >= navigation->count() || requested_page >= pages->count()) return false;
  if (navigation->currentRow() != requested_page) navigation->setCurrentRow(requested_page);
  if (pages->currentIndex() != requested_page) pages->setCurrentIndex(requested_page);
  return navigation->currentRow() == requested_page && pages->currentIndex() == requested_page;
}

}  // namespace workcell_builder

#pragma once
#include <QDialog>
#include <QTableWidget>
#include <QPushButton>
#include <vector>
#include "include/asset_discovery_helper.h"

class AssetPickerDialog : public QDialog {
  Q_OBJECT
public:
  explicit AssetPickerDialog(const QString & title, QWidget * parent = nullptr);
  void set_candidates(const std::vector<AssetCandidate> & candidates, const std::vector<std::string> & searched_paths);
  AssetCandidate selected_candidate() const;
private slots:
  void on_select_clicked();
private:
  QTableWidget * table_;
  QPushButton * select_button_;
  AssetCandidate selected_;
};

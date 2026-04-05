// Copyright 2020 Advanced Remanufacturing and Technology Centre
// Copyright 2020 ROS-Industrial Consortium Asia Pacific Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <QApplication>
#include <iostream>

#include "gui/mainwindow.h"


int main(int argc, char * argv[])
{
  QApplication a(argc, argv);

  a.setStyleSheet(
    /* Base widget font */
    "QWidget {"
    "  font-size: 13px;"
    "}"

    /* Window backgrounds */
    "QMainWindow, QDialog {"
    "  background-color: #f4f6f9;"
    "}"

    /* Primary push buttons */
    "QPushButton {"
    "  background-color: #2563EB;"
    "  color: white;"
    "  border: none;"
    "  border-radius: 5px;"
    "  padding: 7px 16px;"
    "  min-height: 28px;"
    "  font-weight: 600;"
    "}"
    "QPushButton:hover {"
    "  background-color: #1D4ED8;"
    "}"
    "QPushButton:pressed {"
    "  background-color: #1E40AF;"
    "}"
    "QPushButton:disabled {"
    "  background-color: #CBD5E1;"
    "  color: #94A3B8;"
    "}"

    /* Text inputs */
    "QLineEdit {"
    "  background-color: #ffffff;"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  padding: 5px 8px;"
    "  min-height: 24px;"
    "}"
    "QLineEdit:focus {"
    "  border-color: #2563EB;"
    "}"

    /* Read-only text browsers */
    "QTextBrowser {"
    "  background-color: #ffffff;"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  padding: 4px 6px;"
    "}"

    /* Combo boxes */
    "QComboBox {"
    "  background-color: #ffffff;"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  padding: 5px 8px;"
    "  min-height: 24px;"
    "}"
    "QComboBox:focus {"
    "  border-color: #2563EB;"
    "}"
    "QComboBox::drop-down {"
    "  border: none;"
    "  width: 20px;"
    "}"
    "QComboBox QAbstractItemView {"
    "  border: 1px solid #CBD5E1;"
    "  selection-background-color: #2563EB;"
    "  selection-color: white;"
    "  background-color: white;"
    "  outline: 0;"
    "}"

    /* List widgets */
    "QListWidget {"
    "  background-color: #ffffff;"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  outline: none;"
    "}"
    "QListWidget::item {"
    "  padding: 4px 6px;"
    "}"
    "QListWidget::item:selected {"
    "  background-color: #2563EB;"
    "  color: white;"
    "  border-radius: 3px;"
    "}"
    "QListWidget::item:hover:!selected {"
    "  background-color: #EFF6FF;"
    "}"

    /* Check boxes */
    "QCheckBox {"
    "  spacing: 6px;"
    "  color: #374151;"
    "}"
    "QCheckBox::indicator {"
    "  width: 16px;"
    "  height: 16px;"
    "  border: 2px solid #CBD5E1;"
    "  border-radius: 3px;"
    "  background-color: white;"
    "}"
    "QCheckBox::indicator:checked {"
    "  background-color: #2563EB;"
    "  border-color: #2563EB;"
    "}"

    /* Progress bar */
    "QProgressBar {"
    "  border: 1px solid #CBD5E1;"
    "  border-radius: 4px;"
    "  text-align: center;"
    "  background-color: #F1F5F9;"
    "  min-height: 16px;"
    "}"
    "QProgressBar::chunk {"
    "  background-color: #2563EB;"
    "  border-radius: 3px;"
    "}"

    /* Scroll bars */
    "QScrollBar:vertical {"
    "  border: none;"
    "  background-color: #F1F5F9;"
    "  width: 8px;"
    "  margin: 0px;"
    "}"
    "QScrollBar::handle:vertical {"
    "  background-color: #94A3B8;"
    "  border-radius: 4px;"
    "  min-height: 20px;"
    "}"
    "QScrollBar::handle:vertical:hover {"
    "  background-color: #64748B;"
    "}"
    "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {"
    "  height: 0px;"
    "}"
    "QScrollBar:horizontal {"
    "  border: none;"
    "  background-color: #F1F5F9;"
    "  height: 8px;"
    "}"
    "QScrollBar::handle:horizontal {"
    "  background-color: #94A3B8;"
    "  border-radius: 4px;"
    "  min-width: 20px;"
    "}"
    "QScrollBar::handle:horizontal:hover {"
    "  background-color: #64748B;"
    "}"
    "QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {"
    "  width: 0px;"
    "}"

    /* Menu bar */
    "QMenuBar {"
    "  background-color: #1E3A5F;"
    "  color: white;"
    "  padding: 2px;"
    "}"
    "QMenuBar::item:selected {"
    "  background-color: #2563EB;"
    "  border-radius: 3px;"
    "}"

    /* Status bar */
    "QStatusBar {"
    "  background-color: #EFF6FF;"
    "  color: #374151;"
    "  border-top: 1px solid #CBD5E1;"
    "}"
  );

  MainWindow w;
  w.show();
  return a.exec();
}

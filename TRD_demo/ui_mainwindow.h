/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.2.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QTextEdit *textEdit;
    QPushButton *Debug_SingleStep;
    QPushButton *Tracking_start_botton;
    QPushButton *Tracking_Pause_botton;
    QPushButton *Tracking_stop_botton;
    QPushButton *Change_result_botton;
    QWidget *verticalLayoutWidget_2;
    QVBoxLayout *verticalLayout_2;
    QTextEdit *textEdit_2;
    QPushButton *Load_dataset_botton;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(495, 347);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        verticalLayoutWidget = new QWidget(centralWidget);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(300, 10, 181, 278));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        textEdit = new QTextEdit(verticalLayoutWidget);
        textEdit->setObjectName(QStringLiteral("textEdit"));

        verticalLayout->addWidget(textEdit);

        Debug_SingleStep = new QPushButton(verticalLayoutWidget);
        Debug_SingleStep->setObjectName(QStringLiteral("Debug_SingleStep"));

        verticalLayout->addWidget(Debug_SingleStep);

        Tracking_start_botton = new QPushButton(verticalLayoutWidget);
        Tracking_start_botton->setObjectName(QStringLiteral("Tracking_start_botton"));

        verticalLayout->addWidget(Tracking_start_botton);

        Tracking_Pause_botton = new QPushButton(verticalLayoutWidget);
        Tracking_Pause_botton->setObjectName(QStringLiteral("Tracking_Pause_botton"));

        verticalLayout->addWidget(Tracking_Pause_botton);

        Tracking_stop_botton = new QPushButton(verticalLayoutWidget);
        Tracking_stop_botton->setObjectName(QStringLiteral("Tracking_stop_botton"));

        verticalLayout->addWidget(Tracking_stop_botton);

        Change_result_botton = new QPushButton(verticalLayoutWidget);
        Change_result_botton->setObjectName(QStringLiteral("Change_result_botton"));

        verticalLayout->addWidget(Change_result_botton);

        verticalLayoutWidget_2 = new QWidget(centralWidget);
        verticalLayoutWidget_2->setObjectName(QStringLiteral("verticalLayoutWidget_2"));
        verticalLayoutWidget_2->setGeometry(QRect(10, 10, 271, 271));
        verticalLayout_2 = new QVBoxLayout(verticalLayoutWidget_2);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        textEdit_2 = new QTextEdit(verticalLayoutWidget_2);
        textEdit_2->setObjectName(QStringLiteral("textEdit_2"));

        verticalLayout_2->addWidget(textEdit_2);

        Load_dataset_botton = new QPushButton(verticalLayoutWidget_2);
        Load_dataset_botton->setObjectName(QStringLiteral("Load_dataset_botton"));

        verticalLayout_2->addWidget(Load_dataset_botton);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 495, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0));
        textEdit->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Tracker control:</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">When the figure shows up with green bounding box, Click &quot;Start&quot;.</p></body></html>", 0));
        Debug_SingleStep->setText(QApplication::translate("MainWindow", "Debug(Single Step)", 0));
        Tracking_start_botton->setText(QApplication::translate("MainWindow", "Start / continue", 0));
        Tracking_Pause_botton->setText(QApplication::translate("MainWindow", "Pause", 0));
        Tracking_stop_botton->setText(QApplication::translate("MainWindow", "Stop", 0));
        Change_result_botton->setText(QApplication::translate("MainWindow", "Save result", 0));
        textEdit_2->setHtml(QApplication::translate("MainWindow", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Sequence Source:</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">--Dataset folder shoud contain images subfolder &quot;img&quot; and &quot;groundtruth_rect.txt&quot; file. Or user can change the initial bounding box by button &quot;Init file&quot;.</p>\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px;"
                        " margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">--Initial bounding box has to be manually selected for Camera sequence.</p></body></html>", 0));
        Load_dataset_botton->setText(QApplication::translate("MainWindow", "Dataset", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H

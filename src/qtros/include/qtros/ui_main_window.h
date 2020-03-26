/********************************************************************************
** Form generated from reading UI file 'main_window.ui'
**
** Created by: Qt User Interface Compiler version 5.5.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAIN_WINDOW_H
#define UI_MAIN_WINDOW_H

#include <QtCore/QLocale>
#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowDesign
{
public:
    QAction *action_Quit;
    QAction *action_Preferences;
    QAction *actionAbout;
    QAction *actionAbout_Qt;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QTabWidget *tab_manager;
    QWidget *tab_status;
    QVBoxLayout *verticalLayout_2;
    QFrame *frame;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *groupBox;
    QGridLayout *gridLayout;
<<<<<<< HEAD
    QLineEdit *line_edit_host;
    QLineEdit *line_edit_topic;
    QCheckBox *checkbox_use_environment;
    QLabel *label;
    QLabel *label_3;
    QLabel *label_2;
    QLabel *label_4;
    QPushButton *button_connect;
    QCheckBox *checkbox_remember_settings;
    QLineEdit *line_edit_master;
    QPushButton *quit_button;
=======
    QLineEdit *line_edit_master;
    QLabel *label_2;
    QLabel *label;
    QLineEdit *line_edit_host;
    QLabel *label_4;
    QLineEdit *line_edit_topic;
    QLabel *label_3;
    QPushButton *button_connect;
    QPushButton *quit_button;
    QCheckBox *checkbox_use_environment;
    QCheckBox *checkbox_remember_settings;
>>>>>>> dd51b76422fed7cb6366991935500086722711b4
    QWidget *tab;
    QLabel *view_logging;
    QPushButton *startvideo;
    QPushButton *suspendvideo;
    QPushButton *snapshot;
    QWidget *tab_2;
    QLabel *view_logging_2;
    QPushButton *startvideo_2;
    QPushButton *snapshot_2;
    QPushButton *suspendvideo_2;
    QMenuBar *menubar;
    QMenu *menu_File;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindowDesign)
    {
        if (MainWindowDesign->objectName().isEmpty())
            MainWindowDesign->setObjectName(QStringLiteral("MainWindowDesign"));
        MainWindowDesign->resize(1455, 862);
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindowDesign->setWindowIcon(icon);
        MainWindowDesign->setLocale(QLocale(QLocale::English, QLocale::Australia));
        action_Quit = new QAction(MainWindowDesign);
        action_Quit->setObjectName(QStringLiteral("action_Quit"));
        action_Quit->setShortcutContext(Qt::ApplicationShortcut);
        action_Preferences = new QAction(MainWindowDesign);
        action_Preferences->setObjectName(QStringLiteral("action_Preferences"));
        actionAbout = new QAction(MainWindowDesign);
        actionAbout->setObjectName(QStringLiteral("actionAbout"));
        actionAbout_Qt = new QAction(MainWindowDesign);
        actionAbout_Qt->setObjectName(QStringLiteral("actionAbout_Qt"));
        centralwidget = new QWidget(MainWindowDesign);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        tab_manager = new QTabWidget(centralwidget);
        tab_manager->setObjectName(QStringLiteral("tab_manager"));
        tab_manager->setMinimumSize(QSize(100, 0));
        tab_manager->setLocale(QLocale(QLocale::English, QLocale::Australia));
        tab_status = new QWidget();
        tab_status->setObjectName(QStringLiteral("tab_status"));
        verticalLayout_2 = new QVBoxLayout(tab_status);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        frame = new QFrame(tab_status);
        frame->setObjectName(QStringLiteral("frame"));
        QSizePolicy sizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy);
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        verticalLayout_3 = new QVBoxLayout(frame);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        groupBox = new QGroupBox(frame);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        gridLayout = new QGridLayout(groupBox);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
<<<<<<< HEAD
=======
        line_edit_master = new QLineEdit(groupBox);
        line_edit_master->setObjectName(QStringLiteral("line_edit_master"));
        line_edit_master->setMaximumSize(QSize(402, 16777215));

        gridLayout->addWidget(line_edit_master, 2, 0, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setMaximumSize(QSize(400, 30));
        label_2->setFrameShape(QFrame::StyledPanel);
        label_2->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(label_2, 3, 0, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setMaximumSize(QSize(400, 30));
        label->setFrameShape(QFrame::StyledPanel);
        label->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(label, 1, 0, 1, 1);

>>>>>>> dd51b76422fed7cb6366991935500086722711b4
        line_edit_host = new QLineEdit(groupBox);
        line_edit_host->setObjectName(QStringLiteral("line_edit_host"));
        line_edit_host->setMaximumSize(QSize(402, 16777215));

        gridLayout->addWidget(line_edit_host, 4, 0, 1, 1);

<<<<<<< HEAD
        line_edit_topic = new QLineEdit(groupBox);
        line_edit_topic->setObjectName(QStringLiteral("line_edit_topic"));
        line_edit_topic->setEnabled(false);
        line_edit_topic->setMaximumSize(QSize(402, 16777215));

        gridLayout->addWidget(line_edit_topic, 13, 3, 1, 1);
=======
        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 1, 2, 12, 1);
>>>>>>> dd51b76422fed7cb6366991935500086722711b4

        line_edit_topic = new QLineEdit(groupBox);
        line_edit_topic->setObjectName(QStringLiteral("line_edit_topic"));
        line_edit_topic->setEnabled(false);
        line_edit_topic->setMaximumSize(QSize(402, 16777215));

<<<<<<< HEAD
        gridLayout->addWidget(checkbox_use_environment, 11, 3, 1, 1);

        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setMaximumSize(QSize(400, 30));
        label->setFrameShape(QFrame::StyledPanel);
        label->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(label, 1, 0, 1, 1);
=======
        gridLayout->addWidget(line_edit_topic, 12, 3, 1, 1);
>>>>>>> dd51b76422fed7cb6366991935500086722711b4

        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setMaximumSize(QSize(400, 30));
        label_3->setFrameShape(QFrame::StyledPanel);
        label_3->setFrameShadow(QFrame::Raised);

<<<<<<< HEAD
        gridLayout->addWidget(label_3, 12, 3, 1, 1);

        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setMaximumSize(QSize(400, 30));
        label_2->setFrameShape(QFrame::StyledPanel);
        label_2->setFrameShadow(QFrame::Raised);

        gridLayout->addWidget(label_2, 3, 0, 1, 1);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QStringLiteral("label_4"));

        gridLayout->addWidget(label_4, 1, 2, 13, 1);
=======
        gridLayout->addWidget(label_3, 11, 3, 1, 1);
>>>>>>> dd51b76422fed7cb6366991935500086722711b4

        button_connect = new QPushButton(groupBox);
        button_connect->setObjectName(QStringLiteral("button_connect"));
        button_connect->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(button_connect->sizePolicy().hasHeightForWidth());
        button_connect->setSizePolicy(sizePolicy1);
<<<<<<< HEAD
        button_connect->setMaximumSize(QSize(402, 50));

        gridLayout->addWidget(button_connect, 6, 0, 1, 1);

        checkbox_remember_settings = new QCheckBox(groupBox);
        checkbox_remember_settings->setObjectName(QStringLiteral("checkbox_remember_settings"));
        checkbox_remember_settings->setLayoutDirection(Qt::RightToLeft);

        gridLayout->addWidget(checkbox_remember_settings, 10, 3, 1, 1);

        line_edit_master = new QLineEdit(groupBox);
        line_edit_master->setObjectName(QStringLiteral("line_edit_master"));
        line_edit_master->setMaximumSize(QSize(402, 16777215));
=======
        button_connect->setMaximumSize(QSize(402, 16777215));

        gridLayout->addWidget(button_connect, 6, 0, 1, 1);

        quit_button = new QPushButton(groupBox);
        quit_button->setObjectName(QStringLiteral("quit_button"));
        sizePolicy1.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy1);
        quit_button->setMaximumSize(QSize(402, 16777215));

        gridLayout->addWidget(quit_button, 7, 0, 1, 1);

        checkbox_use_environment = new QCheckBox(groupBox);
        checkbox_use_environment->setObjectName(QStringLiteral("checkbox_use_environment"));
        checkbox_use_environment->setLayoutDirection(Qt::RightToLeft);
>>>>>>> dd51b76422fed7cb6366991935500086722711b4

        gridLayout->addWidget(checkbox_use_environment, 10, 3, 1, 1);

<<<<<<< HEAD
        quit_button = new QPushButton(groupBox);
        quit_button->setObjectName(QStringLiteral("quit_button"));
        sizePolicy1.setHeightForWidth(quit_button->sizePolicy().hasHeightForWidth());
        quit_button->setSizePolicy(sizePolicy1);
        quit_button->setMaximumSize(QSize(402, 50));

        gridLayout->addWidget(quit_button, 8, 0, 1, 1);
=======
        checkbox_remember_settings = new QCheckBox(groupBox);
        checkbox_remember_settings->setObjectName(QStringLiteral("checkbox_remember_settings"));
        checkbox_remember_settings->setLayoutDirection(Qt::RightToLeft);

        gridLayout->addWidget(checkbox_remember_settings, 9, 3, 1, 1);
>>>>>>> dd51b76422fed7cb6366991935500086722711b4


        verticalLayout_3->addWidget(groupBox);


        verticalLayout_2->addWidget(frame);

        tab_manager->addTab(tab_status, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        view_logging = new QLabel(tab);
        view_logging->setObjectName(QStringLiteral("view_logging"));
        view_logging->setGeometry(QRect(10, 10, 800, 640));
        view_logging->setMaximumSize(QSize(800, 640));
        view_logging->setFrameShape(QFrame::Box);
        view_logging->setFrameShadow(QFrame::Plain);
        startvideo = new QPushButton(tab);
        startvideo->setObjectName(QStringLiteral("startvideo"));
<<<<<<< HEAD
        startvideo->setGeometry(QRect(840, 10, 221, 61));
        suspendvideo = new QPushButton(tab);
        suspendvideo->setObjectName(QStringLiteral("suspendvideo"));
        suspendvideo->setGeometry(QRect(840, 100, 221, 61));
        snapshot = new QPushButton(tab);
        snapshot->setObjectName(QStringLiteral("snapshot"));
        snapshot->setGeometry(QRect(840, 190, 221, 61));
=======
        startvideo->setGeometry(QRect(10, 590, 171, 41));
        suspendvideo = new QPushButton(tab);
        suspendvideo->setObjectName(QStringLiteral("suspendvideo"));
        suspendvideo->setGeometry(QRect(190, 590, 171, 41));
        snapshot = new QPushButton(tab);
        snapshot->setObjectName(QStringLiteral("snapshot"));
        snapshot->setGeometry(QRect(370, 590, 171, 41));
>>>>>>> dd51b76422fed7cb6366991935500086722711b4
        tab_manager->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QStringLiteral("tab_2"));
        view_logging_2 = new QLabel(tab_2);
        view_logging_2->setObjectName(QStringLiteral("view_logging_2"));
        view_logging_2->setGeometry(QRect(10, 10, 800, 640));
        view_logging_2->setMaximumSize(QSize(800, 640));
        view_logging_2->setFrameShape(QFrame::Box);
        view_logging_2->setFrameShadow(QFrame::Plain);
        startvideo_2 = new QPushButton(tab_2);
        startvideo_2->setObjectName(QStringLiteral("startvideo_2"));
        startvideo_2->setGeometry(QRect(840, 10, 221, 61));
        snapshot_2 = new QPushButton(tab_2);
        snapshot_2->setObjectName(QStringLiteral("snapshot_2"));
        snapshot_2->setGeometry(QRect(840, 190, 221, 61));
        suspendvideo_2 = new QPushButton(tab_2);
        suspendvideo_2->setObjectName(QStringLiteral("suspendvideo_2"));
        suspendvideo_2->setGeometry(QRect(840, 100, 221, 61));
        tab_manager->addTab(tab_2, QString());

        hboxLayout->addWidget(tab_manager);

        MainWindowDesign->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindowDesign);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 1455, 22));
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QStringLiteral("menu_File"));
        MainWindowDesign->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindowDesign);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindowDesign->setStatusBar(statusbar);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(action_Preferences);
        menu_File->addSeparator();
        menu_File->addAction(actionAbout);
        menu_File->addAction(actionAbout_Qt);
        menu_File->addSeparator();
        menu_File->addAction(action_Quit);

        retranslateUi(MainWindowDesign);
        QObject::connect(action_Quit, SIGNAL(triggered()), MainWindowDesign, SLOT(close()));
        QObject::connect(quit_button, SIGNAL(clicked()), MainWindowDesign, SLOT(close()));

        tab_manager->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindowDesign);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowDesign)
    {
        MainWindowDesign->setWindowTitle(QApplication::translate("MainWindowDesign", "QRosApp", 0));
        action_Quit->setText(QApplication::translate("MainWindowDesign", "&Quit", 0));
        action_Quit->setShortcut(QApplication::translate("MainWindowDesign", "Ctrl+Q", 0));
        action_Preferences->setText(QApplication::translate("MainWindowDesign", "&Preferences", 0));
        actionAbout->setText(QApplication::translate("MainWindowDesign", "&About", 0));
        actionAbout_Qt->setText(QApplication::translate("MainWindowDesign", "About &Qt", 0));
        groupBox->setTitle(QString());
<<<<<<< HEAD
        line_edit_host->setText(QApplication::translate("MainWindowDesign", "192.168.1.67", 0));
        line_edit_topic->setText(QApplication::translate("MainWindowDesign", "unused", 0));
        checkbox_use_environment->setText(QApplication::translate("MainWindowDesign", "Use environment variables", 0));
        label->setText(QApplication::translate("MainWindowDesign", "Ros Master Url", 0));
        label_3->setText(QApplication::translate("MainWindowDesign", "Ros Hostname", 0));
        label_2->setText(QApplication::translate("MainWindowDesign", "Ros IP", 0));
        label_4->setText(QString());
=======
        line_edit_master->setText(QApplication::translate("MainWindowDesign", "http://192.168.1.2:11311/", 0));
        label_2->setText(QApplication::translate("MainWindowDesign", "Ros IP", 0));
        label->setText(QApplication::translate("MainWindowDesign", "Ros Master Url", 0));
        line_edit_host->setText(QApplication::translate("MainWindowDesign", "192.168.1.67", 0));
        label_4->setText(QString());
        line_edit_topic->setText(QApplication::translate("MainWindowDesign", "unused", 0));
        label_3->setText(QApplication::translate("MainWindowDesign", "Ros Hostname", 0));
>>>>>>> dd51b76422fed7cb6366991935500086722711b4
#ifndef QT_NO_TOOLTIP
        button_connect->setToolTip(QApplication::translate("MainWindowDesign", "Set the target to the current joint trajectory state.", 0));
#endif // QT_NO_TOOLTIP
#ifndef QT_NO_STATUSTIP
        button_connect->setStatusTip(QApplication::translate("MainWindowDesign", "Clear all waypoints and set the target to the current joint trajectory state.", 0));
#endif // QT_NO_STATUSTIP
        button_connect->setText(QApplication::translate("MainWindowDesign", "Connect", 0));
<<<<<<< HEAD
        checkbox_remember_settings->setText(QApplication::translate("MainWindowDesign", "Remember settings on startup", 0));
        line_edit_master->setText(QApplication::translate("MainWindowDesign", "http://192.168.1.2:11311/", 0));
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0));
=======
        quit_button->setText(QApplication::translate("MainWindowDesign", "Quit", 0));
        checkbox_use_environment->setText(QApplication::translate("MainWindowDesign", "Use environment variables", 0));
        checkbox_remember_settings->setText(QApplication::translate("MainWindowDesign", "Remember settings on startup", 0));
>>>>>>> dd51b76422fed7cb6366991935500086722711b4
        tab_manager->setTabText(tab_manager->indexOf(tab_status), QApplication::translate("MainWindowDesign", "Ros Communications", 0));
        view_logging->setText(QApplication::translate("MainWindowDesign", "cam1", 0));
        startvideo->setText(QApplication::translate("MainWindowDesign", "Start", 0));
        suspendvideo->setText(QApplication::translate("MainWindowDesign", "Stop", 0));
        snapshot->setText(QApplication::translate("MainWindowDesign", "Snapshot", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab), QApplication::translate("MainWindowDesign", "Camera 1", 0));
        view_logging_2->setText(QApplication::translate("MainWindowDesign", "cam2", 0));
        startvideo_2->setText(QApplication::translate("MainWindowDesign", "Start", 0));
        snapshot_2->setText(QApplication::translate("MainWindowDesign", "Snapshot", 0));
        suspendvideo_2->setText(QApplication::translate("MainWindowDesign", "Stop", 0));
        tab_manager->setTabText(tab_manager->indexOf(tab_2), QApplication::translate("MainWindowDesign", "Camera 2", 0));
        menu_File->setTitle(QApplication::translate("MainWindowDesign", "&App", 0));
    } // retranslateUi

};

namespace Ui {
    class MainWindowDesign: public Ui_MainWindowDesign {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAIN_WINDOW_H

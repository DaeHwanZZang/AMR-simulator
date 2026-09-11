#include "gui.hpp"

#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSizePolicy>
#include <QtCore/QString>
#include <QtCore/QTimer>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

#include <sstream>

namespace fms {

namespace {

// -- 색 팔레트 (robot_client/gui.py 라이트 테마 그대로) -----------------------
constexpr const char* BG = "#f2f3f6";
constexpr const char* PANEL_BG = "#ffffff";
constexpr const char* BORDER = "#d7dae1";
constexpr const char* FG = "#20222b";
constexpr const char* FG_DIM = "#6b7080";
constexpr const char* ACCENT = "#2f6fed";
constexpr const char* ESTOP_IDLE = "#c23b52";
constexpr const char* ESTOP_ON = "#e0263f";

std::string state_color(RobotState s) {
    switch (s) {
        case RobotState::IDLE: return "#5b6070";
        case RobotState::BUSY: return "#2f6fed";
        case RobotState::CHARGING: return "#0f9d84";
        case RobotState::ERROR: return "#d92d43";
        case RobotState::EMERGENCY: return "#e0263f";
    }
    return FG;
}

QGroupBox* make_section(QVBoxLayout* parent, const QString& title) {
    auto* box = new QGroupBox(title);
    box->setStyleSheet(QString("QGroupBox { background:%1; border:1px solid %2; border-radius:4px; "
                                "margin-top:10px; font-weight:bold; color:%3; } "
                                "QGroupBox::title { subcontrol-origin: margin; left:8px; padding:0 4px; }")
                            .arg(PANEL_BG, BORDER, FG_DIM));
    parent->addWidget(box);
    return box;
}

QLineEdit* labeled_entry(QVBoxLayout* section, const QString& label, const QString& initial = "") {
    auto* row = new QHBoxLayout();
    auto* l = new QLabel(label);
    l->setMinimumWidth(90);
    l->setStyleSheet(QString("color:%1;").arg(FG_DIM));
    auto* edit = new QLineEdit(initial);
    row->addWidget(l);
    row->addWidget(edit);
    section->addLayout(row);
    return edit;
}

std::optional<double> parse_double(const QString& s) {
    bool ok = false;
    double v = s.trimmed().toDouble(&ok);
    if (!ok) return std::nullopt;
    return v;
}

}  // namespace

static std::string map_sync_color(MapSyncState s) {
    switch (s) {
        case MapSyncState::NONE: return "#6b7080";
        case MapSyncState::SYNCING: return "#b7791f";
        case MapSyncState::SYNCED: return "#0f9d84";
        case MapSyncState::FAILED: return "#d92d43";
    }
    return "#6b7080";
}

void launch_gui(int argc, char** argv, OrderExecutor& executor, std::mutex& exec_mutex,
                 CommandQueue& queue, MapSync& map_sync, double max_velocity, double max_acceleration,
                 double robot_width, double robot_length, std::function<void()> on_close) {
    QApplication app(argc, argv);

    auto* window = new QWidget();
    window->setWindowTitle(QString::fromStdString(executor.robot_id + " Control Panel"));
    window->resize(440, 760);
    window->setStyleSheet(QString("QWidget { background:%1; color:%2; font-size:13px; }").arg(BG, FG));

    auto* outer = new QVBoxLayout(window);
    auto* scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    outer->addWidget(scroll);

    auto* content = new QWidget();
    auto* root = new QVBoxLayout(content);
    scroll->setWidget(content);

    // -- 헤더 --
    auto* header = new QHBoxLayout();
    auto* idLabel = new QLabel(QString::fromStdString(executor.robot_id));
    idLabel->setStyleSheet(QString("font-size:17px; font-weight:bold; color:%1;").arg(FG));
    auto* statusLabel = new QLabel("-");
    statusLabel->setStyleSheet(QString("font-size:17px; font-weight:bold; color:%1;").arg(FG));
    statusLabel->setAlignment(Qt::AlignRight);
    header->addWidget(idLabel);
    header->addStretch();
    header->addWidget(statusLabel);
    root->addLayout(header);

    // -- 물리 패널 --
    auto* panelBox = make_section(root, "  Physical Panel  ");
    auto* panelLayout = new QVBoxLayout(panelBox);
    auto* btnRow = new QHBoxLayout();
    auto* startBtn = new QPushButton("START");
    startBtn->setStyleSheet(QString("background:%1; color:white; font-weight:bold; padding:6px;").arg(ACCENT));
    auto* stopBtn = new QPushButton("STOP");
    stopBtn->setStyleSheet(QString("background:%1; padding:6px;").arg(BORDER));
    btnRow->addWidget(startBtn);
    btnRow->addWidget(stopBtn);
    panelLayout->addLayout(btnRow);

    auto* estopBtn = new QPushButton("\u25a0 E-STOP");
    estopBtn->setStyleSheet(QString("background:%1; color:white; font-weight:bold; padding:10px; margin-top:8px;")
                                 .arg(ESTOP_IDLE));
    panelLayout->addWidget(estopBtn);

    auto* modeRow = new QHBoxLayout();
    auto* modeLabel = new QLabel("Mode");
    modeLabel->setMinimumWidth(90);
    modeLabel->setStyleSheet(QString("color:%1;").arg(FG_DIM));
    auto* autoRadio = new QRadioButton("AUTO");
    auto* manualRadio = new QRadioButton("MANUAL");
    auto* modeGroup = new QButtonGroup(panelBox);
    modeGroup->addButton(autoRadio);
    modeGroup->addButton(manualRadio);
    autoRadio->setChecked(true);
    modeRow->addWidget(modeLabel);
    modeRow->addWidget(autoRadio);
    modeRow->addWidget(manualRadio);
    modeRow->addStretch();
    panelLayout->addLayout(modeRow);

    auto* panelHint = new QLabel("");
    panelHint->setWordWrap(true);
    panelHint->setStyleSheet(QString("color:%1; font-size:11px;").arg(FG_DIM));
    panelLayout->addWidget(panelHint);

    QObject::connect(startBtn, &QPushButton::clicked, [&queue]() { queue.push(CmdPressStart{}); });
    QObject::connect(stopBtn, &QPushButton::clicked, [&queue]() { queue.push(CmdPressStop{}); });
    QObject::connect(estopBtn, &QPushButton::clicked, [&queue, &executor, &exec_mutex]() {
        std::lock_guard<std::mutex> lock(exec_mutex);
        queue.push(CmdEstop{!executor.estop_latched()});
    });
    QObject::connect(autoRadio, &QRadioButton::toggled, [&queue](bool checked) {
        if (checked) queue.push(CmdMode{OperatingMode::AUTO});
    });
    QObject::connect(manualRadio, &QRadioButton::toggled, [&queue](bool checked) {
        if (checked) queue.push(CmdMode{OperatingMode::MANUAL});
    });

    // -- 실시간 상태 --
    auto* infoBox = make_section(root, "  Live Status  ");
    auto* infoLayout = new QVBoxLayout(infoBox);
    auto make_info_row = [&](const QString& label) {
        auto* row = new QHBoxLayout();
        auto* l = new QLabel(label);
        l->setMinimumWidth(90);
        l->setStyleSheet(QString("color:%1;").arg(FG_DIM));
        auto* v = new QLabel("-");
        v->setStyleSheet(QString("font-family: Menlo, monospace; color:%1;").arg(FG));
        row->addWidget(l);
        row->addWidget(v);
        row->addStretch();
        infoLayout->addLayout(row);
        return v;
    };
    QLabel* subStateVal = make_info_row("Sub-state");
    QLabel* xVal = make_info_row("x (m)");
    QLabel* yVal = make_info_row("y (m)");
    QLabel* thetaVal = make_info_row("Theta (rad)");
    QLabel* velVal = make_info_row("Velocity (m/s)");
    QLabel* batteryVal = make_info_row("Battery");
    QLabel* orderVal = make_info_row("Order");
    QLabel* errorsVal = make_info_row("Errors");
    auto* batteryBar = new QProgressBar();
    batteryBar->setRange(0, 100);
    batteryBar->setTextVisible(false);
    infoLayout->addWidget(batteryBar);

    // -- 맵 동기화 (SET_MAP -> 번들 다운로드) --
    auto* mapBox = make_section(root, "  Map Sync  ");
    auto* mapLayout = new QVBoxLayout(mapBox);
    auto make_map_row = [&](const QString& label) {
        auto* row = new QHBoxLayout();
        auto* l = new QLabel(label);
        l->setMinimumWidth(90);
        l->setStyleSheet(QString("color:%1;").arg(FG_DIM));
        auto* v = new QLabel("-");
        v->setStyleSheet(QString("font-family: Menlo, monospace; color:%1;").arg(FG));
        v->setWordWrap(true);
        row->addWidget(l);
        row->addWidget(v, 1);
        mapLayout->addLayout(row);
        return v;
    };
    QLabel* mapStateVal = make_map_row("Sync state");
    QLabel* mapUidVal = make_map_row("Map UID");
    QLabel* mapVersionVal = make_map_row("Version");
    QLabel* mapErrorVal = make_map_row("Error");
    auto* resyncBtnRow = new QHBoxLayout();
    resyncBtnRow->addStretch();
    auto* resyncBtn = new QPushButton("Re-sync");
    resyncBtn->setStyleSheet(QString("background:%1; padding:6px;").arg(BORDER));
    resyncBtnRow->addWidget(resyncBtn);
    mapLayout->addLayout(resyncBtnRow);
    QObject::connect(resyncBtn, &QPushButton::clicked, [&queue]() { queue.push(CmdResyncMap{}); });

    // -- 로봇 설정 (IDLE 전용) --
    auto* settingsBox = make_section(root, "  Robot Settings (IDLE only)  ");
    auto* settingsLayout = new QVBoxLayout(settingsBox);
    QLineEdit* idEdit = labeled_entry(settingsLayout, "Robot ID", QString::fromStdString(executor.robot_id));
    QLineEdit* widthEdit = labeled_entry(settingsLayout, "Width (m)", QString::number(robot_width, 'f', 2));
    QLineEdit* lengthEdit = labeled_entry(settingsLayout, "Length (m)", QString::number(robot_length, 'f', 2));
    QLineEdit* velEdit = labeled_entry(settingsLayout, "Max velocity (m/s)", QString::number(max_velocity, 'f', 3));
    QLineEdit* accelEdit =
        labeled_entry(settingsLayout, "Max acceleration (m/s\u00b2)", QString::number(max_acceleration, 'f', 3));
    auto* settingsBtnRow = new QHBoxLayout();
    settingsBtnRow->addStretch();
    auto* applyBtn = new QPushButton("Apply");
    applyBtn->setStyleSheet(QString("background:%1; color:white; font-weight:bold; padding:6px;").arg(ACCENT));
    settingsBtnRow->addWidget(applyBtn);
    settingsLayout->addLayout(settingsBtnRow);
    auto* settingsStatus = new QLabel("");
    settingsStatus->setStyleSheet(QString("color:%1; font-size:11px;").arg(FG_DIM));
    settingsLayout->addWidget(settingsStatus);

    QObject::connect(applyBtn, &QPushButton::clicked, [=, &queue]() {
        std::string robot_id = idEdit->text().trimmed().toStdString();
        auto width = parse_double(widthEdit->text());
        auto length = parse_double(lengthEdit->text());
        auto vel = parse_double(velEdit->text());
        auto accel = parse_double(accelEdit->text());
        if (robot_id.empty()) {
            settingsStatus->setText("Robot ID cannot be empty");
            return;
        }
        if (!width || *width <= 0 || !length || *length <= 0 || !vel || *vel <= 0 || !accel || *accel <= 0) {
            settingsStatus->setText("Width/length/velocity/acceleration must be positive numbers");
            return;
        }
        queue.push(CmdConfigure{robot_id, *width, *length, *vel, *accel});
        settingsStatus->setText("Settings command sent");
    });

    // -- 이동 명령 --
    auto* moveBox = make_section(root, "  Move Command (destination)  ");
    auto* moveLayout = new QVBoxLayout(moveBox);
    QLineEdit* moveX = labeled_entry(moveLayout, "x (m)");
    QLineEdit* moveY = labeled_entry(moveLayout, "y (m)");
    auto* moveBtnRow = new QHBoxLayout();
    moveBtnRow->addStretch();
    auto* moveBtn = new QPushButton("Move");
    moveBtn->setStyleSheet(QString("background:%1; color:white; font-weight:bold; padding:6px;").arg(ACCENT));
    moveBtnRow->addWidget(moveBtn);
    moveLayout->addLayout(moveBtnRow);
    auto* moveStatus = new QLabel("");
    moveStatus->setStyleSheet(QString("color:%1; font-size:11px;").arg(FG_DIM));
    moveLayout->addWidget(moveStatus);

    QObject::connect(moveBtn, &QPushButton::clicked, [=, &queue]() {
        auto x = parse_double(moveX->text());
        auto y = parse_double(moveY->text());
        if (!x || !y) {
            moveStatus->setText("x, y must be numbers");
            return;
        }
        queue.push(CmdMove{*x, *y});
        std::ostringstream ss;
        ss << "Move command sent: (" << *x << ", " << *y << ")";
        moveStatus->setText(QString::fromStdString(ss.str()));
    });

    // -- 강제 로컬라이징 --
    auto* relocBox = make_section(root, "  Force Relocalize (override pose)  ");
    auto* relocLayout = new QVBoxLayout(relocBox);
    auto* relocWarn = new QLabel("Debug-only feature, no physical robot has this. Drops the current order.");
    relocWarn->setWordWrap(true);
    relocWarn->setStyleSheet("color:#d92d43; font-size:11px;");
    relocLayout->addWidget(relocWarn);
    QLineEdit* relocX = labeled_entry(relocLayout, "x (m)");
    QLineEdit* relocY = labeled_entry(relocLayout, "y (m)");
    QLineEdit* relocTheta = labeled_entry(relocLayout, "theta (rad)", "0.0");
    auto* relocBtnRow = new QHBoxLayout();
    relocBtnRow->addStretch();
    auto* relocBtn = new QPushButton("Force Apply");
    relocBtn->setStyleSheet(QString("background:%1; color:white; font-weight:bold; padding:6px;").arg(ACCENT));
    relocBtnRow->addWidget(relocBtn);
    relocLayout->addLayout(relocBtnRow);
    auto* relocStatus = new QLabel("");
    relocStatus->setStyleSheet(QString("color:%1; font-size:11px;").arg(FG_DIM));
    relocLayout->addWidget(relocStatus);

    QObject::connect(relocBtn, &QPushButton::clicked, [=, &queue]() {
        auto x = parse_double(relocX->text());
        auto y = parse_double(relocY->text());
        auto theta = parse_double(relocTheta->text());
        if (!x || !y || !theta) {
            relocStatus->setText("x, y, theta must be numbers");
            return;
        }
        queue.push(CmdRelocalize{*x, *y, *theta});
        std::ostringstream ss;
        ss << "Relocalize command sent: (" << *x << ", " << *y << ", " << *theta << ")";
        relocStatus->setText(QString::fromStdString(ss.str()));
    });

    root->addStretch();

    // -- 주기 갱신 (200ms, Python REFRESH_MS 와 동일) --
    auto* timer = new QTimer(window);
    QObject::connect(timer, &QTimer::timeout, [=, &executor, &exec_mutex, &map_sync]() {
        State snap;
        bool estop_latched;
        OperatingMode mode;
        bool paused;
        std::optional<PauseSource> pause_src;
        RobotState st;
        {
            std::lock_guard<std::mutex> lock(exec_mutex);
            snap = executor.build_state(0, 0.0);
            estop_latched = executor.estop_latched();
            mode = executor.mode();
            paused = executor.paused();
            pause_src = executor.pause_source();
            st = executor.state();
            if (idLabel->text().toStdString() != executor.robot_id) {
                idLabel->setText(QString::fromStdString(executor.robot_id));
                window->setWindowTitle(QString::fromStdString(executor.robot_id + " Control Panel"));
            }
        }

        std::string display = to_string(snap.state);
        if (snap.sub_state) display += "/" + to_string(*snap.sub_state);
        if (snap.paused && snap.pause_source) display += " (PAUSED:" + to_string(*snap.pause_source) + ")";
        if (snap.mode == OperatingMode::MANUAL) display += " [MANUAL]";
        statusLabel->setText(QString::fromStdString(display));
        statusLabel->setStyleSheet(
            QString("font-size:17px; font-weight:bold; color:%1;").arg(QString::fromStdString(state_color(st))));

        subStateVal->setText(snap.sub_state ? QString::fromStdString(to_string(*snap.sub_state)) : "-");
        xVal->setText(QString::number(snap.pose.x, 'f', 3));
        yVal->setText(QString::number(snap.pose.y, 'f', 3));
        thetaVal->setText(QString::number(snap.pose.theta, 'f', 3));
        velVal->setText(QString::number(snap.velocity, 'f', 3));
        batteryVal->setText(QString::number(snap.battery.charge, 'f', 1) + " %");
        batteryBar->setValue((int)snap.battery.charge);
        orderVal->setText(snap.order_id ? QString::fromStdString(*snap.order_id) : "(none)");
        if (snap.errors.empty()) {
            errorsVal->setText("(none)");
        } else {
            QString joined;
            for (size_t i = 0; i < snap.errors.size(); ++i) {
                if (i) joined += ", ";
                joined += QString::fromStdString(snap.errors[i].error_type);
            }
            errorsVal->setText(joined);
        }

        // 맵 동기화 상태
        MapSync::Status ms = map_sync.status();
        mapStateVal->setText(QString::fromStdString(to_string(ms.state)));
        mapStateVal->setStyleSheet(QString("font-family: Menlo, monospace; font-weight:bold; color:%1;")
                                        .arg(QString::fromStdString(map_sync_color(ms.state))));
        mapUidVal->setText(ms.map_uid.empty() ? "(none)" : QString::fromStdString(ms.map_uid));
        mapVersionVal->setText(ms.map_version.empty() ? "-" : QString::fromStdString(ms.map_version));
        mapErrorVal->setText(ms.error.empty() ? "-" : QString::fromStdString(ms.error));
        resyncBtn->setEnabled(ms.state != MapSyncState::SYNCING);

        // 물리 패널 동기화
        if (estop_latched) {
            estopBtn->setText("\u25a0 E-STOP LATCHED -- press again to release");
            estopBtn->setStyleSheet(
                QString("background:%1; color:white; font-weight:bold; padding:10px; margin-top:8px;")
                    .arg(ESTOP_ON));
        } else {
            estopBtn->setText("\u25a0 E-STOP");
            estopBtn->setStyleSheet(
                QString("background:%1; color:white; font-weight:bold; padding:10px; margin-top:8px;")
                    .arg(ESTOP_IDLE));
        }
        bool block_signals_auto = autoRadio->blockSignals(true);
        bool block_signals_manual = manualRadio->blockSignals(true);
        if (mode == OperatingMode::AUTO) autoRadio->setChecked(true);
        else manualRadio->setChecked(true);
        autoRadio->blockSignals(block_signals_auto);
        manualRadio->blockSignals(block_signals_manual);

        if (paused && pause_src) {
            panelHint->setText(QString::fromStdString("Paused (" + to_string(*pause_src) + ") -- press START to release"));
        } else if (mode == OperatingMode::MANUAL) {
            panelHint->setText("MANUAL -- rejects new jobs and halts motion");
        } else if (estop_latched) {
            panelHint->setText("Releasing the latch resumes the job from where it stopped");
        } else {
            panelHint->setText("");
        }

        // 설정 변경은 IDLE 에서만 (레이스 방지 이중 방어 -- main.cpp 의 CmdConfigure 처리에서도 재검사함)
        bool enabled = (st == RobotState::IDLE);
        idEdit->setEnabled(enabled);
        widthEdit->setEnabled(enabled);
        lengthEdit->setEnabled(enabled);
        velEdit->setEnabled(enabled);
        accelEdit->setEnabled(enabled);
        applyBtn->setEnabled(enabled);
        if (!enabled) settingsStatus->setText("Editable only when IDLE");
        else if (settingsStatus->text() == "Editable only when IDLE") settingsStatus->setText("");
    });
    timer->start(200);

    window->show();
    window->raise();
    window->activateWindow();
    app.exec();
    on_close();
}

}  // namespace fms

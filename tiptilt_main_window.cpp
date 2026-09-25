#include "tiptilt_main_window.h"

#include <string>
#include <utility>

#include <QAction>
#include <QComboBox>
#include <QCloseEvent>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QObject>
#include <QPlainTextEdit>
#include <QPixmap>
#include <QPushButton>
#include <QSpinBox>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <Qt>

#include "tiptilt_config_dialog.h"
#include "tiptilt_config.h"

namespace {

class DarkFlatWorker : public QObject {
    Q_OBJECT
public:
    explicit DarkFlatWorker(TipTiltQtController* controller, DarkFlatUiSettings settings)
        : controller_(controller), settings_(std::move(settings)) {}

public slots:
    void run() {
        std::string errorMessage;
        const bool success = controller_->runDarkFlatCapture(settings_, &errorMessage);
        emit finished(success, QString::fromStdString(success ? "DarkFlat capture completed and files were written." : errorMessage));
    }

signals:
    void finished(bool success, const QString& message);

private:
    TipTiltQtController* controller_ = nullptr;
    DarkFlatUiSettings settings_;
};

class CalibrationWorker : public QObject {
    Q_OBJECT
public:
    explicit CalibrationWorker(TipTiltQtController* controller, CalibrationUiSettings settings)
        : controller_(controller), settings_(std::move(settings)) {}

public slots:
    void run() {
        std::string errorMessage;
        const bool success = controller_->runCalibration(settings_, &errorMessage);
        emit finished(success, QString::fromStdString(success ? "Calibration capture completed." : errorMessage));
    }

signals:
    void finished(bool success, const QString& message);

private:
    TipTiltQtController* controller_ = nullptr;
    CalibrationUiSettings settings_;
};

} // namespace

TipTiltMainWindow::TipTiltMainWindow() {
    buildUi();
    connectActions();
    updateMotorDebugAvailability();

    std::string errorMessage;
    if (controller_.startCameraSession(&errorMessage)) {
        setStatus("Camera preview is running.");
    } else {
        setStatus(QString::fromStdString(errorMessage));
    }

    previewTimer_->start(25);
}

TipTiltMainWindow::~TipTiltMainWindow() {
    if (darkFlatThread_ != nullptr) {
        darkFlatThread_->quit();
        darkFlatThread_->wait();
    }
    if (calibrationThread_ != nullptr) {
        calibrationThread_->quit();
        calibrationThread_->wait();
    }
    controller_.shutdown();
}

void TipTiltMainWindow::closeEvent(QCloseEvent* event) {
    if (darkFlatThread_ != nullptr || calibrationThread_ != nullptr) {
        QMessageBox::warning(this, "Background Work Running", "Wait for calibration or DarkFlat capture to finish before closing the window.");
        event->ignore();
        return;
    }
    controller_.shutdown();
    QMainWindow::closeEvent(event);
}

void TipTiltMainWindow::buildUi() {
    setWindowTitle("TipTilt Control");
    resize(1280, 860);
    auto* fileMenu = menuBar()->addMenu("File");
    editConfigAction_ = fileMenu->addAction("Edit Config");

    auto* centralWidget = new QWidget(this);
    auto* mainLayout = new QVBoxLayout(centralWidget);

    previewLabel_ = new QLabel("Waiting for camera frames...", centralWidget);
    previewLabel_->setMinimumSize(900, 600);
    previewLabel_->setAlignment(Qt::AlignCenter);
    previewLabel_->setStyleSheet("background-color: #111; color: #ddd; border: 1px solid #444;");

    auto* controlsLayout = new QHBoxLayout();
    auto* formLayout = new QFormLayout();

    exposureSpinBox_ = new QDoubleSpinBox(centralWidget);
    exposureSpinBox_->setDecimals(2);
    exposureSpinBox_->setRange(0.0, 1000000.0);
    exposureSpinBox_->setValue(controller_.settings().camera.exposureTime);
    exposureSpinBox_->setSuffix(" us");

    targetFrameRateSpinBox_ = new QDoubleSpinBox(centralWidget);
    targetFrameRateSpinBox_->setDecimals(2);
    targetFrameRateSpinBox_->setRange(0.0, 1000.0);
    targetFrameRateSpinBox_->setValue(controller_.settings().camera.targetFrameRate);
    targetFrameRateSpinBox_->setSuffix(" fps");

    nucModeLineEdit_ = new QLineEdit(QString::fromStdString(controller_.settings().camera.nucMode), centralWidget);

    xComSpinBox_ = new QSpinBox(centralWidget);
    xComSpinBox_->setRange(1, 64);
    xComSpinBox_->setValue(controller_.settings().xComPort);

    yComSpinBox_ = new QSpinBox(centralWidget);
    yComSpinBox_->setRange(1, 64);
    yComSpinBox_->setValue(controller_.settings().yComPort);

    autoguiderComSpinBox_ = new QSpinBox(centralWidget);
    autoguiderComSpinBox_->setRange(1, 64);
    autoguiderComSpinBox_->setValue(controller_.settings().autoguiderComPort);

    autoguiderMotorFrequencySpinBox_ = new QSpinBox(centralWidget);
    autoguiderMotorFrequencySpinBox_->setRange(1, 100000);
    autoguiderMotorFrequencySpinBox_->setValue(controller_.settings().autoguiderMotorFrequency);

    refreshSpinBox_ = new QSpinBox(centralWidget);
    refreshSpinBox_->setRange(1, 100000);
    refreshSpinBox_->setValue(controller_.settings().referenceRefreshFrames);

    acquisitionModeSpinBox_ = new QSpinBox(centralWidget);
    acquisitionModeSpinBox_->setRange(1, 2);
    acquisitionModeSpinBox_->setValue(controller_.settings().acquisitionMode);

    formLayout->addRow("Exposure", exposureSpinBox_);
    formLayout->addRow("Camera FPS", targetFrameRateSpinBox_);
    formLayout->addRow("NUC Mode", nucModeLineEdit_);
    formLayout->addRow("TipTilt X COM", xComSpinBox_);
    formLayout->addRow("TipTilt Y COM", yComSpinBox_);
    formLayout->addRow("Autoguider COM", autoguiderComSpinBox_);
    formLayout->addRow("Autoguider Freq", autoguiderMotorFrequencySpinBox_);
    formLayout->addRow("Reference Refresh", refreshSpinBox_);
    formLayout->addRow("Acquisition Mode", acquisitionModeSpinBox_);

    initializeTipTiltButton_ = new QPushButton("Initialize TipTilt", centralWidget);
    initializeAutoguiderButton_ = new QPushButton("Initialize Autoguider", centralWidget);
    startLoopButton_ = new QPushButton("Start Loop", centralWidget);

    auto* buttonLayout = new QVBoxLayout();
    buttonLayout->addWidget(initializeTipTiltButton_);
    buttonLayout->addWidget(initializeAutoguiderButton_);
    buttonLayout->addWidget(startLoopButton_);
    buttonLayout->addStretch();

    controlsLayout->addLayout(formLayout, 2);
    controlsLayout->addLayout(buttonLayout, 1);

    auto* manualVoltageGroup = new QGroupBox("VoltageTest", centralWidget);
    auto* manualVoltageLayout = new QFormLayout(manualVoltageGroup);
    manualVoltageXSpinBox_ = new QDoubleSpinBox(manualVoltageGroup);
    manualVoltageYSpinBox_ = new QDoubleSpinBox(manualVoltageGroup);
    manualVoltageXSpinBox_->setRange(-20.0, 130.0);
    manualVoltageYSpinBox_->setRange(-20.0, 130.0);
    manualVoltageXSpinBox_->setDecimals(3);
    manualVoltageYSpinBox_->setDecimals(3);
    applyVoltageButton_ = new QPushButton("Apply Manual Voltages", manualVoltageGroup);
    manualVoltageLayout->addRow("X Voltage", manualVoltageXSpinBox_);
    manualVoltageLayout->addRow("Y Voltage", manualVoltageYSpinBox_);
    manualVoltageLayout->addRow(applyVoltageButton_);

    auto* utilityGroup = new QGroupBox("Utility Tools", centralWidget);
    auto* utilityLayout = new QFormLayout(utilityGroup);
    runBinningBenchmarkButton_ = new QPushButton("Run Image Binning Benchmark", utilityGroup);
    mklIterationsSpinBox_ = new QSpinBox(utilityGroup);
    mklIterationsSpinBox_->setRange(1, 1000000);
    mklIterationsSpinBox_->setValue(3000);
    runMklBenchmarkButton_ = new QPushButton("Run MKL FFT Benchmark", utilityGroup);
    matrixSkipValuesSpinBox_ = new QSpinBox(utilityGroup);
    matrixSkipValuesSpinBox_->setRange(0, 1000000);
    matrixSkipValuesSpinBox_->setValue(0);
    matrixSampleCountSpinBox_ = new QSpinBox(utilityGroup);
    matrixSampleCountSpinBox_->setRange(1, 1000000);
    matrixSampleCountSpinBox_->setValue(100);
    buildVoltageShiftMatrixButton_ = new QPushButton("Build Voltage Shift Matrix", utilityGroup);
    darkFlatFramesSpinBox_ = new QSpinBox(utilityGroup);
    darkFlatFramesSpinBox_->setRange(1, 100000);
    darkFlatFramesSpinBox_->setValue(100);
    darkFlatXStartSpinBox_ = new QSpinBox(utilityGroup);
    darkFlatXStartSpinBox_->setRange(-1000, 1000);
    darkFlatXStartSpinBox_->setValue(-20);
    darkFlatXEndSpinBox_ = new QSpinBox(utilityGroup);
    darkFlatXEndSpinBox_->setRange(-1000, 1000);
    darkFlatXEndSpinBox_->setValue(130);
    darkFlatYStartSpinBox_ = new QSpinBox(utilityGroup);
    darkFlatYStartSpinBox_->setRange(-1000, 1000);
    darkFlatYStartSpinBox_->setValue(-20);
    darkFlatYEndSpinBox_ = new QSpinBox(utilityGroup);
    darkFlatYEndSpinBox_->setRange(-1000, 1000);
    darkFlatYEndSpinBox_->setValue(130);
    darkFlatStepSpinBox_ = new QSpinBox(utilityGroup);
    darkFlatStepSpinBox_->setRange(1, 1000);
    darkFlatStepSpinBox_->setValue(5);
    darkFlatSettleTimeSpinBox_ = new QSpinBox(utilityGroup);
    darkFlatSettleTimeSpinBox_->setRange(0, 100000);
    darkFlatSettleTimeSpinBox_->setValue(50);
    calibrationFramesSpinBox_ = new QSpinBox(utilityGroup);
    calibrationFramesSpinBox_->setRange(2, 100000);
    calibrationFramesSpinBox_->setValue(100);
    calibrationFramesPerPositionSpinBox_ = new QSpinBox(utilityGroup);
    calibrationFramesPerPositionSpinBox_->setRange(1, 1000);
    calibrationFramesPerPositionSpinBox_->setValue(1);
    calibrationVoltageStartSpinBox_ = new QDoubleSpinBox(utilityGroup);
    calibrationVoltageStartSpinBox_->setRange(-130.0, 130.0);
    calibrationVoltageStartSpinBox_->setValue(-10.0);
    calibrationVoltageStartSpinBox_->setDecimals(3);
    calibrationVoltageRangeSpinBox_ = new QDoubleSpinBox(utilityGroup);
    calibrationVoltageRangeSpinBox_->setRange(-130.0, 130.0);
    calibrationVoltageRangeSpinBox_->setValue(20.0);
    calibrationVoltageRangeSpinBox_->setDecimals(3);
    calibrationStationaryVoltageSpinBox_ = new QDoubleSpinBox(utilityGroup);
    calibrationStationaryVoltageSpinBox_->setRange(-130.0, 130.0);
    calibrationStationaryVoltageSpinBox_->setValue(0.0);
    calibrationStationaryVoltageSpinBox_->setDecimals(3);
    calibrationMotorComSpinBox_ = new QSpinBox(utilityGroup);
    calibrationMotorComSpinBox_->setRange(1, 64);
    calibrationMotorComSpinBox_->setValue(1);
    calibrationMotorFrequencySpinBox_ = new QSpinBox(utilityGroup);
    calibrationMotorFrequencySpinBox_->setRange(1, 100000);
    calibrationMotorFrequencySpinBox_->setValue(350);
    serialDebugComSpinBox_ = new QSpinBox(utilityGroup);
    serialDebugComSpinBox_->setRange(1, 64);
    serialDebugComSpinBox_->setValue(1);
    serialDebugBaudSpinBox_ = new QSpinBox(utilityGroup);
    serialDebugBaudSpinBox_->setRange(1, 10000000);
    serialDebugBaudSpinBox_->setValue(9600);
    serialDebugReadBytesSpinBox_ = new QSpinBox(utilityGroup);
    serialDebugReadBytesSpinBox_->setRange(0, 4096);
    serialDebugReadBytesSpinBox_->setValue(0);
    motorDebugControllerComSpinBox_ = new QSpinBox(utilityGroup);
    motorDebugControllerComSpinBox_->setRange(1, 64);
    motorDebugControllerComSpinBox_->setValue(1);
    motorDebugMotorSpinBox_ = new QSpinBox(utilityGroup);
    motorDebugMotorSpinBox_->setRange(1, 8);
    motorDebugMotorSpinBox_->setValue(1);
    motorDebugFrequencySpinBox_ = new QSpinBox(utilityGroup);
    motorDebugFrequencySpinBox_->setRange(1, 100000);
    motorDebugFrequencySpinBox_->setValue(350);
    motorDebugCountsSpinBox_ = new QSpinBox(utilityGroup);
    motorDebugCountsSpinBox_->setRange(0, 1000000);
    motorDebugCountsSpinBox_->setValue(100);
    calibrationModeComboBox_ = new QComboBox(utilityGroup);
    calibrationModeComboBox_->addItem("Actuator", 0);
    calibrationModeComboBox_->addItem("Motor", 1);
    calibrationAxisComboBox_ = new QComboBox(utilityGroup);
    calibrationAxisComboBox_->addItem("Both Axes", -1);
    calibrationAxisComboBox_->addItem("X Axis", 0);
    calibrationAxisComboBox_->addItem("Y Axis", 1);
    motorDebugDirectionComboBox_ = new QComboBox(utilityGroup);
    motorDebugDirectionComboBox_->addItem("Positive", 1);
    motorDebugDirectionComboBox_->addItem("Negative", -1);
    darkFlatOutputDirectoryLineEdit_ = new QLineEdit("Flats", utilityGroup);
    darkFlatFilePrefixLineEdit_ = new QLineEdit("MeanFlat", utilityGroup);
    serialDebugPayloadLineEdit_ = new QLineEdit(utilityGroup);
    serialDebugPayloadLineEdit_->setPlaceholderText("command payload");
    runDarkFlatWorkflowButton_ = new QPushButton("Run DarkFlat Capture", utilityGroup);
    runCalibrationWorkflowButton_ = new QPushButton("Run Calibration Capture", utilityGroup);
    runSerialDebugButton_ = new QPushButton("Run Serial Debug", utilityGroup);
    connectMotorDebugButton_ = new QPushButton("Connect Motor Controller", utilityGroup);
    disconnectMotorDebugButton_ = new QPushButton("Disconnect Motor Controller", utilityGroup);
    enableMotorDebugButton_ = new QPushButton("Enable Motor", utilityGroup);
    disableMotorDebugButton_ = new QPushButton("Disable Motor", utilityGroup);
    setMotorFrequencyDebugButton_ = new QPushButton("Set Motor Frequency", utilityGroup);
    moveMotorDebugButton_ = new QPushButton("Move Motor", utilityGroup);
    exitMotorDebugButton_ = new QPushButton("Exit Motor", utilityGroup);
    utilityLayout->addRow(runBinningBenchmarkButton_);
    utilityLayout->addRow("MKL Iterations", mklIterationsSpinBox_);
    utilityLayout->addRow(runMklBenchmarkButton_);
    utilityLayout->addRow("Matrix Skip Values", matrixSkipValuesSpinBox_);
    utilityLayout->addRow("Matrix Sample Count", matrixSampleCountSpinBox_);
    utilityLayout->addRow(buildVoltageShiftMatrixButton_);
    utilityLayout->addRow("DarkFlat Frames/Pos", darkFlatFramesSpinBox_);
    utilityLayout->addRow("DarkFlat X Start", darkFlatXStartSpinBox_);
    utilityLayout->addRow("DarkFlat X End", darkFlatXEndSpinBox_);
    utilityLayout->addRow("DarkFlat Y Start", darkFlatYStartSpinBox_);
    utilityLayout->addRow("DarkFlat Y End", darkFlatYEndSpinBox_);
    utilityLayout->addRow("DarkFlat Step", darkFlatStepSpinBox_);
    utilityLayout->addRow("DarkFlat Settle ms", darkFlatSettleTimeSpinBox_);
    utilityLayout->addRow("DarkFlat Output Dir", darkFlatOutputDirectoryLineEdit_);
    utilityLayout->addRow("DarkFlat Prefix", darkFlatFilePrefixLineEdit_);
    utilityLayout->addRow(runDarkFlatWorkflowButton_);
    utilityLayout->addRow("Calibration Mode", calibrationModeComboBox_);
    utilityLayout->addRow("Calibration Axis", calibrationAxisComboBox_);
    utilityLayout->addRow("Calibration Frames", calibrationFramesSpinBox_);
    utilityLayout->addRow("Frames / Position", calibrationFramesPerPositionSpinBox_);
    utilityLayout->addRow("Calibration Start V", calibrationVoltageStartSpinBox_);
    utilityLayout->addRow("Calibration Range V", calibrationVoltageRangeSpinBox_);
    utilityLayout->addRow("Stationary Axis V", calibrationStationaryVoltageSpinBox_);
    utilityLayout->addRow("Motor Controller COM", calibrationMotorComSpinBox_);
    utilityLayout->addRow("Motor Frequency", calibrationMotorFrequencySpinBox_);
    utilityLayout->addRow(runCalibrationWorkflowButton_);
    utilityLayout->addRow("Serial Debug COM", serialDebugComSpinBox_);
    utilityLayout->addRow("Serial Debug Baud", serialDebugBaudSpinBox_);
    utilityLayout->addRow("Serial Read Bytes", serialDebugReadBytesSpinBox_);
    utilityLayout->addRow("Serial Payload", serialDebugPayloadLineEdit_);
    utilityLayout->addRow(runSerialDebugButton_);
    utilityLayout->addRow("Motor Debug COM", motorDebugControllerComSpinBox_);
    utilityLayout->addRow(connectMotorDebugButton_);
    utilityLayout->addRow(disconnectMotorDebugButton_);
    utilityLayout->addRow("Motor Number", motorDebugMotorSpinBox_);
    utilityLayout->addRow("Motor Frequency", motorDebugFrequencySpinBox_);
    utilityLayout->addRow(setMotorFrequencyDebugButton_);
    utilityLayout->addRow(enableMotorDebugButton_);
    utilityLayout->addRow(disableMotorDebugButton_);
    utilityLayout->addRow("Move Direction", motorDebugDirectionComboBox_);
    utilityLayout->addRow("Move Counts", motorDebugCountsSpinBox_);
    utilityLayout->addRow(moveMotorDebugButton_);
    utilityLayout->addRow(exitMotorDebugButton_);

    toolOutput_ = new QPlainTextEdit(centralWidget);
    toolOutput_->setReadOnly(true);
    toolOutput_->setPlaceholderText("Tool output from the unified TipTilt Qt app will appear here.");

    statusLabel_ = new QLabel("Idle", centralWidget);
    statusLabel_->setWordWrap(true);

    mainLayout->addWidget(previewLabel_, 1);
    mainLayout->addLayout(controlsLayout);
    mainLayout->addWidget(manualVoltageGroup);
    mainLayout->addWidget(utilityGroup);
    mainLayout->addWidget(toolOutput_, 1);
    mainLayout->addWidget(statusLabel_);

    previewTimer_ = new QTimer(this);
    setCentralWidget(centralWidget);
}

void TipTiltMainWindow::connectActions() {
    connect(previewTimer_, &QTimer::timeout, this, [this]() {
        refreshPreview();
        updateMotorDebugAvailability();
    });

    connect(editConfigAction_, &QAction::triggered, this, [this]() {
        openConfigEditor();
    });

    connect(initializeTipTiltButton_, &QPushButton::clicked, this, [this]() {
        syncSettingsFromInputs();
        std::string errorMessage;
        if (controller_.initializeTipTilt(&errorMessage)) {
            initializeTipTiltButton_->setEnabled(false);
            setStatus("Tip/tilt hardware initialized.");
        } else {
            setStatus(QString::fromStdString(errorMessage));
        }
    });

    connect(initializeAutoguiderButton_, &QPushButton::clicked, this, [this]() {
        syncSettingsFromInputs();
        std::string errorMessage;
        if (controller_.initializeAutoguider(&errorMessage)) {
            initializeAutoguiderButton_->setEnabled(false);
            setStatus("Autoguider hardware initialized.");
        } else {
            setStatus(QString::fromStdString(errorMessage));
        }
        updateMotorDebugAvailability();
    });

    connect(startLoopButton_, &QPushButton::clicked, this, [this]() {
        syncSettingsFromInputs();
        std::string errorMessage;
        if (controller_.startClosedLoop(&errorMessage)) {
            startLoopButton_->setEnabled(false);
            setStatus("Closed loop started.");
        } else {
            setStatus(QString::fromStdString(errorMessage));
        }
        updateMotorDebugAvailability();
    });

    connect(applyVoltageButton_, &QPushButton::clicked, this, [this]() {
        std::string errorMessage;
        if (controller_.applyManualTipTiltVoltages(manualVoltageXSpinBox_->value(), manualVoltageYSpinBox_->value(), &errorMessage)) {
            appendToolOutput(QString("VoltageTest: applied X=%1 Y=%2").arg(manualVoltageXSpinBox_->value()).arg(manualVoltageYSpinBox_->value()));
            setStatus("Manual tip/tilt voltages applied.");
        } else {
            appendToolOutput(QString::fromStdString(errorMessage));
            setStatus(QString::fromStdString(errorMessage));
        }
    });

    connect(runBinningBenchmarkButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(controller_.runImageBinningBenchmark());
        appendToolOutput(message);
        setStatus(message);
    });

    connect(runMklBenchmarkButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(controller_.runMklBenchmark(mklIterationsSpinBox_->value()));
        appendToolOutput(message);
        setStatus(message);
    });

    connect(buildVoltageShiftMatrixButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(
            controller_.runVoltageShiftMatrixFit(matrixSkipValuesSpinBox_->value(), matrixSampleCountSpinBox_->value())
        );
        appendToolOutput(message);
        setStatus(message);
    });

    connect(runDarkFlatWorkflowButton_, &QPushButton::clicked, this, [this]() {
        startDarkFlatCapture();
    });

    connect(runSerialDebugButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(
            controller_.runSerialDebug(
                serialDebugComSpinBox_->value(),
                serialDebugBaudSpinBox_->value(),
                serialDebugPayloadLineEdit_->text().toStdString(),
                serialDebugReadBytesSpinBox_->value()
            )
        );
        appendToolOutput(message);
        setStatus(message);
    });

    connect(runCalibrationWorkflowButton_, &QPushButton::clicked, this, [this]() {
        if (calibrationThread_ != nullptr) {
            const QString message = "Calibration capture is already running.";
            appendToolOutput(message);
            setStatus(message);
            return;
        }

        CalibrationUiSettings settings;
        settings.frames = calibrationFramesSpinBox_->value();
        settings.framesPerPosition = calibrationFramesPerPositionSpinBox_->value();
        settings.mode = calibrationModeComboBox_->currentData().toInt();
        settings.axis = calibrationAxisComboBox_->currentData().toInt();
        settings.voltageStart = calibrationVoltageStartSpinBox_->value();
        settings.voltageRange = calibrationVoltageRangeSpinBox_->value();
        settings.stationaryVoltage = calibrationStationaryVoltageSpinBox_->value();
        settings.motorControllerComPort = calibrationMotorComSpinBox_->value();
        settings.motorFrequency = calibrationMotorFrequencySpinBox_->value();

        calibrationThread_ = new QThread(this);
        auto* worker = new CalibrationWorker(&controller_, settings);
        worker->moveToThread(calibrationThread_);

        connect(calibrationThread_, &QThread::started, worker, &CalibrationWorker::run);
        connect(worker, &CalibrationWorker::finished, this, [this](bool success, const QString& message) {
            runCalibrationWorkflowButton_->setEnabled(true);
            appendToolOutput(message);
            setStatus(message);
            if (!success) {
                QMessageBox::warning(this, "Calibration Capture", message);
            }
        });
        connect(worker, &CalibrationWorker::finished, calibrationThread_, &QThread::quit);
        connect(worker, &CalibrationWorker::finished, worker, &QObject::deleteLater);
        connect(calibrationThread_, &QThread::finished, this, [this]() {
            if (calibrationThread_ != nullptr) {
                calibrationThread_->deleteLater();
                calibrationThread_ = nullptr;
            }
        });

        runCalibrationWorkflowButton_->setEnabled(false);
        appendToolOutput("Calibration capture started in background.");
        setStatus("Calibration capture is running...");
        calibrationThread_->start();
    });

    connect(connectMotorDebugButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(
            controller_.connectMotorController(motorDebugControllerComSpinBox_->value())
        );
        appendToolOutput(message);
        setStatus(message);
    });

    connect(disconnectMotorDebugButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(controller_.disconnectMotorController());
        appendToolOutput(message);
        setStatus(message);
    });

    connect(setMotorFrequencyDebugButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(
            controller_.setMotorFrequency(
                motorDebugMotorSpinBox_->value(),
                motorDebugFrequencySpinBox_->value()
            )
        );
        appendToolOutput(message);
        setStatus(message);
    });

    connect(enableMotorDebugButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(
            controller_.setMotorEnabled(motorDebugMotorSpinBox_->value(), true)
        );
        appendToolOutput(message);
        setStatus(message);
    });

    connect(disableMotorDebugButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(
            controller_.setMotorEnabled(motorDebugMotorSpinBox_->value(), false)
        );
        appendToolOutput(message);
        setStatus(message);
    });

    connect(moveMotorDebugButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(
            controller_.moveMotor(
                motorDebugMotorSpinBox_->value(),
                motorDebugDirectionComboBox_->currentData().toInt(),
                motorDebugCountsSpinBox_->value()
            )
        );
        appendToolOutput(message);
        setStatus(message);
    });

    connect(exitMotorDebugButton_, &QPushButton::clicked, this, [this]() {
        const QString message = QString::fromStdString(
            controller_.exitMotor(motorDebugMotorSpinBox_->value())
        );
        appendToolOutput(message);
        setStatus(message);
    });
}

void TipTiltMainWindow::syncSettingsFromInputs() {
    controller_.settings().camera.exposureTime = exposureSpinBox_->value();
    controller_.settings().camera.targetFrameRate = targetFrameRateSpinBox_->value();
    controller_.settings().camera.nucMode = nucModeLineEdit_->text().toStdString();
    controller_.settings().xComPort = xComSpinBox_->value();
    controller_.settings().yComPort = yComSpinBox_->value();
    controller_.settings().autoguiderComPort = autoguiderComSpinBox_->value();
    controller_.settings().autoguiderMotorFrequency = autoguiderMotorFrequencySpinBox_->value();
    controller_.settings().referenceRefreshFrames = refreshSpinBox_->value();
    controller_.settings().acquisitionMode = acquisitionModeSpinBox_->value();
}

void TipTiltMainWindow::refreshPreview() {
    const QImage image = controller_.capturePreviewImage();
    if (image.isNull()) {
        return;
    }

    previewLabel_->setPixmap(QPixmap::fromImage(image).scaled(
        previewLabel_->size(),
        Qt::KeepAspectRatio,
        Qt::SmoothTransformation
    ));
}

void TipTiltMainWindow::openConfigEditor() {
    TipTiltConfigDialog dialog(this);
    if (dialog.exec() == QDialog::Accepted) {
        try {
            const TipTiltConfig config = LoadTipTiltConfig(GetDefaultTipTiltConfigPath());
            controller_.settings().xComPort = config.tipTiltXComPort;
            controller_.settings().yComPort = config.tipTiltYComPort;
            controller_.settings().autoguiderComPort = config.autoguiderComPort;
            controller_.settings().autoguiderMotorFrequency = config.autoguiderMotorFrequency;
            controller_.settings().camera.targetFrameRate = config.cameraTargetFrameRate;
            controller_.settings().camera.exposureTime = config.cameraExposureTime;
            controller_.settings().camera.nucMode = config.cameraNucMode;

            xComSpinBox_->setValue(config.tipTiltXComPort);
            yComSpinBox_->setValue(config.tipTiltYComPort);
            autoguiderComSpinBox_->setValue(config.autoguiderComPort);
            autoguiderMotorFrequencySpinBox_->setValue(config.autoguiderMotorFrequency);
            targetFrameRateSpinBox_->setValue(config.cameraTargetFrameRate);
            exposureSpinBox_->setValue(config.cameraExposureTime);
            nucModeLineEdit_->setText(QString::fromStdString(config.cameraNucMode));
        } catch (...) {
        }
        setStatus("Configuration saved to CalibrationMatrix.csv.");
    }
}

void TipTiltMainWindow::startDarkFlatCapture() {
    if (darkFlatThread_ != nullptr) {
        const QString message = "DarkFlat capture is already running.";
        appendToolOutput(message);
        setStatus(message);
        return;
    }

    DarkFlatUiSettings settings;
    settings.framesPerPosition = darkFlatFramesSpinBox_->value();
    settings.xStart = darkFlatXStartSpinBox_->value();
    settings.xEnd = darkFlatXEndSpinBox_->value();
    settings.yStart = darkFlatYStartSpinBox_->value();
    settings.yEnd = darkFlatYEndSpinBox_->value();
    settings.step = darkFlatStepSpinBox_->value();
    settings.settleTimeMs = darkFlatSettleTimeSpinBox_->value();
    settings.outputDirectory = darkFlatOutputDirectoryLineEdit_->text().toStdString();
    settings.filePrefix = darkFlatFilePrefixLineEdit_->text().toStdString();

    darkFlatThread_ = new QThread(this);
    auto* worker = new DarkFlatWorker(&controller_, settings);
    worker->moveToThread(darkFlatThread_);

    connect(darkFlatThread_, &QThread::started, worker, &DarkFlatWorker::run);
    connect(worker, &DarkFlatWorker::finished, this, [this](bool success, const QString& message) {
        runDarkFlatWorkflowButton_->setEnabled(true);
        appendToolOutput(message);
        setStatus(message);
        if (!success) {
            QMessageBox::warning(this, "DarkFlat Capture", message);
        }
    });
    connect(worker, &DarkFlatWorker::finished, darkFlatThread_, &QThread::quit);
    connect(worker, &DarkFlatWorker::finished, worker, &QObject::deleteLater);
    connect(darkFlatThread_, &QThread::finished, this, [this]() {
        if (darkFlatThread_ != nullptr) {
            darkFlatThread_->deleteLater();
            darkFlatThread_ = nullptr;
        }
    });

    runDarkFlatWorkflowButton_->setEnabled(false);
    appendToolOutput("DarkFlat capture started in background.");
    setStatus("DarkFlat capture is running...");
    darkFlatThread_->start();
}

void TipTiltMainWindow::appendToolOutput(const QString& message) {
    toolOutput_->appendPlainText(message);
}

void TipTiltMainWindow::setStatus(const QString& message) {
    statusLabel_->setText(message);
}

void TipTiltMainWindow::updateMotorDebugAvailability() {
    const bool autoguiderOwnsController = controller_.isAutoguiderInitialized();
    connectMotorDebugButton_->setEnabled(!autoguiderOwnsController);
    disconnectMotorDebugButton_->setEnabled(!autoguiderOwnsController);
    enableMotorDebugButton_->setEnabled(!autoguiderOwnsController);
    disableMotorDebugButton_->setEnabled(!autoguiderOwnsController);
    setMotorFrequencyDebugButton_->setEnabled(!autoguiderOwnsController);
    moveMotorDebugButton_->setEnabled(!autoguiderOwnsController);
    exitMotorDebugButton_->setEnabled(!autoguiderOwnsController);
    motorDebugControllerComSpinBox_->setEnabled(!autoguiderOwnsController);
    motorDebugMotorSpinBox_->setEnabled(!autoguiderOwnsController);
    motorDebugFrequencySpinBox_->setEnabled(!autoguiderOwnsController);
    motorDebugCountsSpinBox_->setEnabled(!autoguiderOwnsController);
    motorDebugDirectionComboBox_->setEnabled(!autoguiderOwnsController);

    if (autoguiderOwnsController) {
        connectMotorDebugButton_->setToolTip("Disabled while autoguider hardware owns the motor controller connection.");
        disconnectMotorDebugButton_->setToolTip("Disabled while autoguider hardware owns the motor controller connection.");
        enableMotorDebugButton_->setToolTip("Disabled while autoguider hardware owns the motor controller connection.");
        disableMotorDebugButton_->setToolTip("Disabled while autoguider hardware owns the motor controller connection.");
        setMotorFrequencyDebugButton_->setToolTip("Disabled while autoguider hardware owns the motor controller connection.");
        moveMotorDebugButton_->setToolTip("Disabled while autoguider hardware owns the motor controller connection.");
        exitMotorDebugButton_->setToolTip("Disabled while autoguider hardware owns the motor controller connection.");
    } else {
        connectMotorDebugButton_->setToolTip({});
        disconnectMotorDebugButton_->setToolTip({});
        enableMotorDebugButton_->setToolTip({});
        disableMotorDebugButton_->setToolTip({});
        setMotorFrequencyDebugButton_->setToolTip({});
        moveMotorDebugButton_->setToolTip({});
        exitMotorDebugButton_->setToolTip({});
    }
}

#include "tiptilt_main_window.moc"

#include "tiptilt_config_dialog.h"

#include <exception>
#include <string>

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QSpinBox>
#include <QVBoxLayout>
#include <QWidget>

#include "controls.h"

namespace {

QDoubleSpinBox* createDoubleSpinBox(QWidget* parent, double minValue = -1000000.0, double maxValue = 1000000.0, int decimals = 6) {
    auto* spinBox = new QDoubleSpinBox(parent);
    spinBox->setRange(minValue, maxValue);
    spinBox->setDecimals(decimals);
    spinBox->setSingleStep(0.01);
    return spinBox;
}

QSpinBox* createIntSpinBox(QWidget* parent, int minValue = -1000000, int maxValue = 1000000) {
    auto* spinBox = new QSpinBox(parent);
    spinBox->setRange(minValue, maxValue);
    return spinBox;
}

QGroupBox* createMatrixGroup(
    const QString& title,
    QWidget* parent,
    const std::array<const char*, 4>& labels,
    std::array<QDoubleSpinBox*, 4>& spinBoxes
) {
    auto* groupBox = new QGroupBox(title, parent);
    auto* layout = new QGridLayout(groupBox);
    for (int index = 0; index < 4; ++index) {
        auto* label = new QLabel(labels[index], groupBox);
        spinBoxes[index] = createDoubleSpinBox(groupBox);
        layout->addWidget(label, index / 2, (index % 2) * 2);
        layout->addWidget(spinBoxes[index], index / 2, (index % 2) * 2 + 1);
    }
    return groupBox;
}

} // namespace

TipTiltConfigDialog::TipTiltConfigDialog(QWidget* parent) : QDialog(parent) {
    buildUi();
    loadFromDisk();
}

void TipTiltConfigDialog::buildUi() {
    setWindowTitle("Edit TipTilt Config");
    resize(880, 760);

    auto* rootLayout = new QVBoxLayout(this);
    auto* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);

    auto* content = new QWidget(scrollArea);
    auto* contentLayout = new QVBoxLayout(content);

    contentLayout->addWidget(createMatrixGroup(
        "Correction Matrix",
        content,
        {"A00", "A10", "A01", "A11"},
        correctionMatrix_
    ));
    contentLayout->addWidget(createMatrixGroup(
        "Calibration Matrix",
        content,
        {"ClM00", "ClM10", "ClM01", "ClM11"},
        calibrationMatrix_
    ));

    auto* hardwareGroup = new QGroupBox("Hardware And Camera Defaults", content);
    auto* hardwareLayout = new QFormLayout(hardwareGroup);
    tipTiltXComPortSpinBox_ = createIntSpinBox(hardwareGroup, 0, 256);
    tipTiltYComPortSpinBox_ = createIntSpinBox(hardwareGroup, 0, 256);
    autoguiderComPortSpinBox_ = createIntSpinBox(hardwareGroup, 0, 256);
    autoguiderMotorFrequencySpinBox_ = createIntSpinBox(hardwareGroup, 1, 100000);
    cameraTargetFrameRateSpinBox_ = createDoubleSpinBox(hardwareGroup, 0.0, 1000.0, 3);
    cameraExposureTimeSpinBox_ = createDoubleSpinBox(hardwareGroup, 0.0, 1000000.0, 3);
    cameraNucModeLineEdit_ = new QLineEdit(hardwareGroup);
    hardwareLayout->addRow("TipTilt X COM", tipTiltXComPortSpinBox_);
    hardwareLayout->addRow("TipTilt Y COM", tipTiltYComPortSpinBox_);
    hardwareLayout->addRow("Autoguider COM", autoguiderComPortSpinBox_);
    hardwareLayout->addRow("Autoguider Frequency", autoguiderMotorFrequencySpinBox_);
    hardwareLayout->addRow("Camera FPS", cameraTargetFrameRateSpinBox_);
    hardwareLayout->addRow("Camera Exposure", cameraExposureTimeSpinBox_);
    hardwareLayout->addRow("Camera NUC Mode", cameraNucModeLineEdit_);
    contentLayout->addWidget(hardwareGroup);

    auto* tipTiltGroup = new QGroupBox("TipTilt Control", content);
    auto* tipTiltLayout = new QFormLayout(tipTiltGroup);
    vxOffsetSpinBox_ = createDoubleSpinBox(tipTiltGroup);
    vyOffsetSpinBox_ = createDoubleSpinBox(tipTiltGroup);
    slewRateSpinBox_ = createDoubleSpinBox(tipTiltGroup);
    tipTiltPid_[0] = createDoubleSpinBox(tipTiltGroup);
    tipTiltPid_[1] = createDoubleSpinBox(tipTiltGroup);
    tipTiltPid_[2] = createDoubleSpinBox(tipTiltGroup);
    integralWindowSpinBox_ = createIntSpinBox(tipTiltGroup);
    derivativeWindowSpinBox_ = createIntSpinBox(tipTiltGroup);
    tipTiltLayout->addRow("Vx Offset", vxOffsetSpinBox_);
    tipTiltLayout->addRow("Vy Offset", vyOffsetSpinBox_);
    tipTiltLayout->addRow("Slew Rate", slewRateSpinBox_);
    tipTiltLayout->addRow("Kp", tipTiltPid_[0]);
    tipTiltLayout->addRow("Kd", tipTiltPid_[1]);
    tipTiltLayout->addRow("Ki", tipTiltPid_[2]);
    tipTiltLayout->addRow("Integral Window (Ni)", integralWindowSpinBox_);
    tipTiltLayout->addRow("Derivative Window (Nd)", derivativeWindowSpinBox_);
    contentLayout->addWidget(tipTiltGroup);

    contentLayout->addWidget(createMatrixGroup(
        "Autoguider Matrix",
        content,
        {"AA00", "AA10", "AA01", "AA11"},
        autoguiderMatrix_
    ));

    auto* autoguiderGroup = new QGroupBox("Autoguider Control", content);
    auto* autoguiderLayout = new QFormLayout(autoguiderGroup);
    autoguiderPid_[0] = createDoubleSpinBox(autoguiderGroup);
    autoguiderPid_[1] = createDoubleSpinBox(autoguiderGroup);
    autoguiderPid_[2] = createDoubleSpinBox(autoguiderGroup);
    autoguiderCorrectionTimeSpinBox_ = createIntSpinBox(autoguiderGroup, 0, 1000000);
    imageSaveAfterSecondsSpinBox_ = createIntSpinBox(autoguiderGroup, 0, 1000000);
    autoguiderOffloadXSpinBox_ = createDoubleSpinBox(autoguiderGroup);
    autoguiderOffloadYSpinBox_ = createDoubleSpinBox(autoguiderGroup);
    autoguiderLayout->addRow("Autoguider Kp", autoguiderPid_[0]);
    autoguiderLayout->addRow("Autoguider Kd", autoguiderPid_[1]);
    autoguiderLayout->addRow("Autoguider Ki", autoguiderPid_[2]);
    autoguiderLayout->addRow("Correction Time (frames)", autoguiderCorrectionTimeSpinBox_);
    autoguiderLayout->addRow("Save Image Every (s)", imageSaveAfterSecondsSpinBox_);
    autoguiderLayout->addRow("Offload Limit X", autoguiderOffloadXSpinBox_);
    autoguiderLayout->addRow("Offload Limit Y", autoguiderOffloadYSpinBox_);
    contentLayout->addWidget(autoguiderGroup);

    auto* filterGroup = new QGroupBox("Filter And Limits", content);
    auto* filterLayout = new QFormLayout(filterGroup);
    sampleTimeSpinBox_ = createDoubleSpinBox(filterGroup, 0.0, 1000.0, 8);
    outputMinSpinBox_ = createDoubleSpinBox(filterGroup);
    outputMaxSpinBox_ = createDoubleSpinBox(filterGroup);
    integratorMinSpinBox_ = createDoubleSpinBox(filterGroup);
    integratorMaxSpinBox_ = createDoubleSpinBox(filterGroup);
    derivativeTauSpinBox_ = createDoubleSpinBox(filterGroup, 0.0, 1000.0, 8);
    derivativeCutoffSpinBox_ = createDoubleSpinBox(filterGroup, 0.0, 100000.0, 4);
    filterLayout->addRow("Sample Time", sampleTimeSpinBox_);
    filterLayout->addRow("Output Min", outputMinSpinBox_);
    filterLayout->addRow("Output Max", outputMaxSpinBox_);
    filterLayout->addRow("Integrator Min", integratorMinSpinBox_);
    filterLayout->addRow("Integrator Max", integratorMaxSpinBox_);
    filterLayout->addRow("Derivative Tau", derivativeTauSpinBox_);
    filterLayout->addRow("Derivative Cutoff", derivativeCutoffSpinBox_);
    contentLayout->addWidget(filterGroup);

    auto* modeGroup = new QGroupBox("Modes", content);
    auto* modeLayout = new QFormLayout(modeGroup);
    showLiveViewCheckBox_ = new QCheckBox("Show live view", modeGroup);
    useCameraFlatCheckBox_ = new QCheckBox("Use camera flat", modeGroup);
    autoPModeCheckBox_ = new QCheckBox("Enable Auto-P mode", modeGroup);
    autoPBounds_[0] = createDoubleSpinBox(modeGroup);
    autoPBounds_[1] = createDoubleSpinBox(modeGroup);
    autoPBounds_[2] = createDoubleSpinBox(modeGroup);
    autoPBounds_[3] = createDoubleSpinBox(modeGroup);
    modeLayout->addRow(showLiveViewCheckBox_);
    modeLayout->addRow(useCameraFlatCheckBox_);
    modeLayout->addRow(autoPModeCheckBox_);
    modeLayout->addRow("Min Auto Kp", autoPBounds_[0]);
    modeLayout->addRow("Max Auto Kp", autoPBounds_[1]);
    modeLayout->addRow("Min Auto Kd", autoPBounds_[2]);
    modeLayout->addRow("Max Auto Kd", autoPBounds_[3]);
    contentLayout->addWidget(modeGroup);
    contentLayout->addStretch();

    scrollArea->setWidget(content);
    rootLayout->addWidget(scrollArea, 1);

    auto* buttonBox = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    connect(buttonBox, &QDialogButtonBox::accepted, this, [this]() {
        saveToDisk();
    });
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    rootLayout->addWidget(buttonBox);
}

void TipTiltConfigDialog::loadFromDisk() {
    try {
        populateFields(LoadTipTiltConfig(GetDefaultTipTiltConfigPath()));
    } catch (const std::exception& ex) {
        QMessageBox::warning(this, "Config Load Failed", QString::fromStdString(ex.what()));
    }
}

void TipTiltConfigDialog::populateFields(const TipTiltConfig& config) {
    tipTiltXComPortSpinBox_->setValue(config.tipTiltXComPort);
    tipTiltYComPortSpinBox_->setValue(config.tipTiltYComPort);
    autoguiderComPortSpinBox_->setValue(config.autoguiderComPort);
    autoguiderMotorFrequencySpinBox_->setValue(config.autoguiderMotorFrequency);
    cameraTargetFrameRateSpinBox_->setValue(config.cameraTargetFrameRate);
    cameraExposureTimeSpinBox_->setValue(config.cameraExposureTime);
    cameraNucModeLineEdit_->setText(QString::fromStdString(config.cameraNucMode));

    correctionMatrix_[0]->setValue(config.correctionA00);
    correctionMatrix_[1]->setValue(config.correctionA10);
    correctionMatrix_[2]->setValue(config.correctionA01);
    correctionMatrix_[3]->setValue(config.correctionA11);

    calibrationMatrix_[0]->setValue(config.calibrationClM00);
    calibrationMatrix_[1]->setValue(config.calibrationClM10);
    calibrationMatrix_[2]->setValue(config.calibrationClM01);
    calibrationMatrix_[3]->setValue(config.calibrationClM11);

    vxOffsetSpinBox_->setValue(config.vxOffset);
    vyOffsetSpinBox_->setValue(config.vyOffset);
    slewRateSpinBox_->setValue(config.slewRate);
    tipTiltPid_[0]->setValue(config.tipTiltKp);
    tipTiltPid_[1]->setValue(config.tipTiltKd);
    tipTiltPid_[2]->setValue(config.tipTiltKi);
    integralWindowSpinBox_->setValue(config.integralWindow);
    derivativeWindowSpinBox_->setValue(config.derivativeWindow);

    autoguiderMatrix_[0]->setValue(config.autoguiderA00);
    autoguiderMatrix_[1]->setValue(config.autoguiderA10);
    autoguiderMatrix_[2]->setValue(config.autoguiderA01);
    autoguiderMatrix_[3]->setValue(config.autoguiderA11);

    autoguiderPid_[0]->setValue(config.autoguiderKp);
    autoguiderPid_[1]->setValue(config.autoguiderKd);
    autoguiderPid_[2]->setValue(config.autoguiderKi);
    autoguiderCorrectionTimeSpinBox_->setValue(config.autoguiderCorrectionTime);
    imageSaveAfterSecondsSpinBox_->setValue(config.imageSaveAfterSeconds);
    autoguiderOffloadXSpinBox_->setValue(config.autoguiderOffloadLimitX);
    autoguiderOffloadYSpinBox_->setValue(config.autoguiderOffloadLimitY);

    sampleTimeSpinBox_->setValue(config.sampleTime);
    outputMinSpinBox_->setValue(config.outputMin);
    outputMaxSpinBox_->setValue(config.outputMax);
    integratorMinSpinBox_->setValue(config.integratorMin);
    integratorMaxSpinBox_->setValue(config.integratorMax);
    derivativeTauSpinBox_->setValue(config.derivativeTau);
    derivativeCutoffSpinBox_->setValue(config.derivativeCutoffFrequency);

    showLiveViewCheckBox_->setChecked(config.showLiveView);
    useCameraFlatCheckBox_->setChecked(config.useCameraFlat);
    autoPModeCheckBox_->setChecked(config.autoPMode);
    autoPBounds_[0]->setValue(config.minAutoKp);
    autoPBounds_[1]->setValue(config.maxAutoKp);
    autoPBounds_[2]->setValue(config.minAutoKd);
    autoPBounds_[3]->setValue(config.maxAutoKd);
}

TipTiltConfig TipTiltConfigDialog::readFields() const {
    TipTiltConfig config;
    config.tipTiltXComPort = tipTiltXComPortSpinBox_->value();
    config.tipTiltYComPort = tipTiltYComPortSpinBox_->value();
    config.autoguiderComPort = autoguiderComPortSpinBox_->value();
    config.autoguiderMotorFrequency = autoguiderMotorFrequencySpinBox_->value();
    config.cameraTargetFrameRate = cameraTargetFrameRateSpinBox_->value();
    config.cameraExposureTime = cameraExposureTimeSpinBox_->value();
    config.cameraNucMode = cameraNucModeLineEdit_->text().toStdString();

    config.correctionA00 = correctionMatrix_[0]->value();
    config.correctionA10 = correctionMatrix_[1]->value();
    config.correctionA01 = correctionMatrix_[2]->value();
    config.correctionA11 = correctionMatrix_[3]->value();

    config.calibrationClM00 = calibrationMatrix_[0]->value();
    config.calibrationClM10 = calibrationMatrix_[1]->value();
    config.calibrationClM01 = calibrationMatrix_[2]->value();
    config.calibrationClM11 = calibrationMatrix_[3]->value();

    config.vxOffset = vxOffsetSpinBox_->value();
    config.vyOffset = vyOffsetSpinBox_->value();
    config.slewRate = slewRateSpinBox_->value();
    config.tipTiltKp = tipTiltPid_[0]->value();
    config.tipTiltKd = tipTiltPid_[1]->value();
    config.tipTiltKi = tipTiltPid_[2]->value();
    config.integralWindow = integralWindowSpinBox_->value();
    config.derivativeWindow = derivativeWindowSpinBox_->value();

    config.autoguiderA00 = autoguiderMatrix_[0]->value();
    config.autoguiderA10 = autoguiderMatrix_[1]->value();
    config.autoguiderA01 = autoguiderMatrix_[2]->value();
    config.autoguiderA11 = autoguiderMatrix_[3]->value();

    config.autoguiderKp = autoguiderPid_[0]->value();
    config.autoguiderKd = autoguiderPid_[1]->value();
    config.autoguiderKi = autoguiderPid_[2]->value();
    config.autoguiderCorrectionTime = autoguiderCorrectionTimeSpinBox_->value();
    config.imageSaveAfterSeconds = imageSaveAfterSecondsSpinBox_->value();
    config.autoguiderOffloadLimitX = autoguiderOffloadXSpinBox_->value();
    config.autoguiderOffloadLimitY = autoguiderOffloadYSpinBox_->value();

    config.sampleTime = sampleTimeSpinBox_->value();
    config.outputMin = outputMinSpinBox_->value();
    config.outputMax = outputMaxSpinBox_->value();
    config.integratorMin = integratorMinSpinBox_->value();
    config.integratorMax = integratorMaxSpinBox_->value();
    config.derivativeTau = derivativeTauSpinBox_->value();
    config.derivativeCutoffFrequency = derivativeCutoffSpinBox_->value();

    config.showLiveView = showLiveViewCheckBox_->isChecked();
    config.useCameraFlat = useCameraFlatCheckBox_->isChecked();
    config.autoPMode = autoPModeCheckBox_->isChecked();
    config.minAutoKp = autoPBounds_[0]->value();
    config.maxAutoKp = autoPBounds_[1]->value();
    config.minAutoKd = autoPBounds_[2]->value();
    config.maxAutoKd = autoPBounds_[3]->value();
    return config;
}

void TipTiltConfigDialog::saveToDisk() {
    std::string errorMessage;
    const TipTiltConfig config = readFields();
    if (!SaveTipTiltConfig(config, GetDefaultTipTiltConfigPath(), &errorMessage)) {
        QMessageBox::critical(this, "Config Save Failed", QString::fromStdString(errorMessage));
        return;
    }

    ApplyTipTiltConfigToGlobals(config);
    accept();
}

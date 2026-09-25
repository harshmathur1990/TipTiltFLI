#ifndef TIPTILT_CONFIG_DIALOG_H
#define TIPTILT_CONFIG_DIALOG_H

#include <array>

#include <QDialog>

class QCheckBox;
class QDoubleSpinBox;
class QLineEdit;
class QSpinBox;

#include "tiptilt_config.h"

class TipTiltConfigDialog : public QDialog {
public:
    explicit TipTiltConfigDialog(QWidget* parent = nullptr);

private:
    void buildUi();
    void loadFromDisk();
    void populateFields(const TipTiltConfig& config);
    TipTiltConfig readFields() const;
    void saveToDisk();

    std::array<QDoubleSpinBox*, 4> correctionMatrix_{};
    std::array<QDoubleSpinBox*, 4> calibrationMatrix_{};
    std::array<QDoubleSpinBox*, 4> autoguiderMatrix_{};
    std::array<QDoubleSpinBox*, 3> tipTiltPid_{};
    std::array<QDoubleSpinBox*, 3> autoguiderPid_{};
    std::array<QDoubleSpinBox*, 4> autoPBounds_{};

    QSpinBox* tipTiltXComPortSpinBox_ = nullptr;
    QSpinBox* tipTiltYComPortSpinBox_ = nullptr;
    QSpinBox* autoguiderComPortSpinBox_ = nullptr;
    QSpinBox* autoguiderMotorFrequencySpinBox_ = nullptr;
    QDoubleSpinBox* cameraTargetFrameRateSpinBox_ = nullptr;
    QDoubleSpinBox* cameraExposureTimeSpinBox_ = nullptr;
    QLineEdit* cameraNucModeLineEdit_ = nullptr;
    QDoubleSpinBox* vxOffsetSpinBox_ = nullptr;
    QDoubleSpinBox* vyOffsetSpinBox_ = nullptr;
    QDoubleSpinBox* slewRateSpinBox_ = nullptr;
    QSpinBox* integralWindowSpinBox_ = nullptr;
    QSpinBox* derivativeWindowSpinBox_ = nullptr;
    QSpinBox* autoguiderCorrectionTimeSpinBox_ = nullptr;
    QSpinBox* imageSaveAfterSecondsSpinBox_ = nullptr;
    QDoubleSpinBox* sampleTimeSpinBox_ = nullptr;
    QDoubleSpinBox* outputMinSpinBox_ = nullptr;
    QDoubleSpinBox* outputMaxSpinBox_ = nullptr;
    QDoubleSpinBox* integratorMinSpinBox_ = nullptr;
    QDoubleSpinBox* integratorMaxSpinBox_ = nullptr;
    QDoubleSpinBox* derivativeTauSpinBox_ = nullptr;
    QDoubleSpinBox* autoguiderOffloadXSpinBox_ = nullptr;
    QDoubleSpinBox* autoguiderOffloadYSpinBox_ = nullptr;
    QDoubleSpinBox* derivativeCutoffSpinBox_ = nullptr;
    QCheckBox* showLiveViewCheckBox_ = nullptr;
    QCheckBox* useCameraFlatCheckBox_ = nullptr;
    QCheckBox* autoPModeCheckBox_ = nullptr;
};

#endif

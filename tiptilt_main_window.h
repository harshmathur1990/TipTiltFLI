#ifndef TIPTILT_MAIN_WINDOW_H
#define TIPTILT_MAIN_WINDOW_H

#include <QMainWindow>

class QLabel;
class QPushButton;
class QSpinBox;
class QDoubleSpinBox;
class QComboBox;
class QLineEdit;
class QPlainTextEdit;
class QThread;
class QTimer;
class QAction;

#include "tiptilt_qt_controller.h"

class TipTiltMainWindow : public QMainWindow {
public:
    TipTiltMainWindow();
    ~TipTiltMainWindow() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void buildUi();
    void connectActions();
    void syncSettingsFromInputs();
    void refreshPreview();
    void openConfigEditor();
    void startDarkFlatCapture();
    void updateMotorDebugAvailability();
    void appendToolOutput(const QString& message);
    void setStatus(const QString& message);

    TipTiltQtController controller_;
    QLabel* previewLabel_ = nullptr;
    QLabel* statusLabel_ = nullptr;
    QDoubleSpinBox* exposureSpinBox_ = nullptr;
    QDoubleSpinBox* targetFrameRateSpinBox_ = nullptr;
    QSpinBox* xComSpinBox_ = nullptr;
    QSpinBox* yComSpinBox_ = nullptr;
    QSpinBox* autoguiderComSpinBox_ = nullptr;
    QSpinBox* autoguiderMotorFrequencySpinBox_ = nullptr;
    QSpinBox* refreshSpinBox_ = nullptr;
    QSpinBox* acquisitionModeSpinBox_ = nullptr;
    QSpinBox* mklIterationsSpinBox_ = nullptr;
    QSpinBox* matrixSkipValuesSpinBox_ = nullptr;
    QSpinBox* matrixSampleCountSpinBox_ = nullptr;
    QSpinBox* darkFlatFramesSpinBox_ = nullptr;
    QSpinBox* darkFlatXStartSpinBox_ = nullptr;
    QSpinBox* darkFlatXEndSpinBox_ = nullptr;
    QSpinBox* darkFlatYStartSpinBox_ = nullptr;
    QSpinBox* darkFlatYEndSpinBox_ = nullptr;
    QSpinBox* darkFlatStepSpinBox_ = nullptr;
    QSpinBox* darkFlatSettleTimeSpinBox_ = nullptr;
    QSpinBox* calibrationFramesSpinBox_ = nullptr;
    QSpinBox* calibrationFramesPerPositionSpinBox_ = nullptr;
    QDoubleSpinBox* calibrationVoltageStartSpinBox_ = nullptr;
    QDoubleSpinBox* calibrationVoltageRangeSpinBox_ = nullptr;
    QDoubleSpinBox* calibrationStationaryVoltageSpinBox_ = nullptr;
    QSpinBox* calibrationMotorComSpinBox_ = nullptr;
    QSpinBox* calibrationMotorFrequencySpinBox_ = nullptr;
    QSpinBox* serialDebugComSpinBox_ = nullptr;
    QSpinBox* serialDebugBaudSpinBox_ = nullptr;
    QSpinBox* serialDebugReadBytesSpinBox_ = nullptr;
    QSpinBox* motorDebugControllerComSpinBox_ = nullptr;
    QSpinBox* motorDebugMotorSpinBox_ = nullptr;
    QSpinBox* motorDebugFrequencySpinBox_ = nullptr;
    QSpinBox* motorDebugCountsSpinBox_ = nullptr;
    QComboBox* calibrationModeComboBox_ = nullptr;
    QComboBox* calibrationAxisComboBox_ = nullptr;
    QComboBox* motorDebugDirectionComboBox_ = nullptr;
    QLineEdit* nucModeLineEdit_ = nullptr;
    QLineEdit* darkFlatOutputDirectoryLineEdit_ = nullptr;
    QLineEdit* darkFlatFilePrefixLineEdit_ = nullptr;
    QLineEdit* serialDebugPayloadLineEdit_ = nullptr;
    QPushButton* initializeTipTiltButton_ = nullptr;
    QPushButton* initializeAutoguiderButton_ = nullptr;
    QPushButton* startLoopButton_ = nullptr;
    QPushButton* applyVoltageButton_ = nullptr;
    QPushButton* runBinningBenchmarkButton_ = nullptr;
    QPushButton* runMklBenchmarkButton_ = nullptr;
    QPushButton* buildVoltageShiftMatrixButton_ = nullptr;
    QPushButton* runDarkFlatWorkflowButton_ = nullptr;
    QPushButton* runCalibrationWorkflowButton_ = nullptr;
    QPushButton* runSerialDebugButton_ = nullptr;
    QPushButton* connectMotorDebugButton_ = nullptr;
    QPushButton* disconnectMotorDebugButton_ = nullptr;
    QPushButton* enableMotorDebugButton_ = nullptr;
    QPushButton* disableMotorDebugButton_ = nullptr;
    QPushButton* setMotorFrequencyDebugButton_ = nullptr;
    QPushButton* moveMotorDebugButton_ = nullptr;
    QPushButton* exitMotorDebugButton_ = nullptr;
    QDoubleSpinBox* manualVoltageXSpinBox_ = nullptr;
    QDoubleSpinBox* manualVoltageYSpinBox_ = nullptr;
    QPlainTextEdit* toolOutput_ = nullptr;
    QThread* darkFlatThread_ = nullptr;
    QThread* calibrationThread_ = nullptr;
    QTimer* previewTimer_ = nullptr;
    QAction* editConfigAction_ = nullptr;
};

#endif

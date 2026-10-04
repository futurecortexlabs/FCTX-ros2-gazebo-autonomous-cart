# ファイルの役割: 速度スライダーと数値入力を同期するQt部品。入力の変更通知が循環しないようにする。
"""Slider and numeric entry that stay in sync without feedback loops."""
from PyQt5 import QtCore, QtWidgets


# スライダーと数値欄を一組として扱う速度入力部品。
class SpeedControl(QtWidgets.QWidget):
    valueChanged = QtCore.pyqtSignal(float)

    # このクラスで使う状態・通信先・画面部品を初期化する。
    def __init__(self, title, maximum, value, unit='m/s'):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(3)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(QtWidgets.QLabel(title))
        row.addStretch()
        self.spin = QtWidgets.QDoubleSpinBox()
        self.spin.setRange(0., maximum)
        self.spin.setDecimals(2)
        self.spin.setSingleStep(.05)
        self.spin.setSuffix(' '+unit)
        self.spin.setKeyboardTracking(False)
        self.spin.setMinimumWidth(128)
        row.addWidget(self.spin)
        layout.addLayout(row)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, round(maximum*100))
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)
        self.slider.setFocusPolicy(QtCore.Qt.NoFocus)
        layout.addWidget(self.slider)
        self.slider.valueChanged.connect(lambda v: self.set_value(v/100))
        self.spin.valueChanged.connect(self.set_value)
        self.set_value(value, emit=False)

    # 現在の数値欄の値を、速度の実数として返す。
    def value(self):
        return self.spin.value()

    # 範囲と小数桁をそろえ、信号を一時停止してスライダーと数値欄を同期する。
    def set_value(self, value, emit=True):
        value = max(self.spin.minimum(), min(self.spin.maximum(), round(float(value), 2)))
        # 同期によるvalueChangedの再帰的な発火を抑え、最後に必要な通知だけを出す。
        blockers = [QtCore.QSignalBlocker(self.slider), QtCore.QSignalBlocker(self.spin)]
        self.slider.setValue(round(value*100))
        self.spin.setValue(value)
        del blockers
        if emit:
            self.valueChanged.emit(value)

    # 数値入力中またはドラッグ中かを返し、外部の読出し値で編集中の値を上書きしないようにする。
    def editing(self):
        return self.spin.hasFocus() or self.spin.lineEdit().hasFocus() or self.slider.isSliderDown()

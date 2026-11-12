"""不依赖DAC值的ADC三通道独立传递曲线校准。"""

import bisect
import json
import math
from pathlib import Path


SUPPORTED_SCHEMA_VERSION = 1
SUPPORTED_CALIBRATION_TYPE = "per_channel_piecewise_linear"
EXPECTED_CHANNEL_ORDER = ["ADC0", "ADC1", "ADC2"]


class AdcCalibration:
    """经过完整校验、加载后不可变的三通道ADC传递曲线。"""

    def __init__(self, document, source_path=None):
        self.document = document
        self.source_path = Path(source_path).resolve() if source_path else None
        self._validate()
        self.calibration_id = document["calibration_id"]
        self.endpoint_tolerance_v = float(
            document["validity"]["endpoint_tolerance_v"]
        )
        self.curves = tuple(tuple(curve["nodes"]) for curve in document["curves"])
        self.raw_nodes = tuple(
            tuple(node["raw_voltage_v"] for node in curve) for curve in self.curves
        )
        self.input_ranges = tuple((nodes[0], nodes[-1]) for nodes in self.raw_nodes)

    @classmethod
    def load(cls, path):
        path = Path(path)
        try:
            with path.open(encoding="utf-8") as config_file:
                document = json.load(config_file)
        except json.JSONDecodeError as exc:
            raise ValueError(f"校准JSON格式错误：{exc}") from exc
        if not isinstance(document, dict):
            raise ValueError("校准配置顶层必须是JSON对象")
        return cls(document, path)

    @staticmethod
    def _finite_number(value, name):
        if isinstance(value, (bool, str, bytes)):
            raise ValueError(f"{name}必须是数字")
        try:
            value = float(value)
        except (TypeError, ValueError, OverflowError) as exc:
            raise ValueError(f"{name}必须是数字") from exc
        if not math.isfinite(value):
            raise ValueError(f"{name}必须是有限数字")
        return value

    def _validate(self):
        document = self.document
        if document.get("schema_version") != SUPPORTED_SCHEMA_VERSION:
            raise ValueError("不支持的校准schema_version")
        if document.get("calibration_type") != SUPPORTED_CALIBRATION_TYPE:
            raise ValueError(
                "仅支持不依赖DAC的per_channel_piecewise_linear校准配置"
            )
        if (
            not isinstance(document.get("calibration_id"), str)
            or not document["calibration_id"]
        ):
            raise ValueError("calibration_id不能为空")
        if document.get("channel_order") != EXPECTED_CHANNEL_ORDER:
            raise ValueError("channel_order必须是ADC0、ADC1、ADC2")

        processing = document.get("processing")
        if not isinstance(processing, dict):
            raise ValueError("缺少processing配置")
        if processing.get("interpolation") != "piecewise_linear_per_channel":
            raise ValueError("仅支持每通道分段线性插值")
        if processing.get("runtime_dac_dependency") is not False:
            raise ValueError("ADC独立校准配置不得依赖DAC值")
        if processing.get("offset_correction") is not False:
            raise ValueError("当前版本不支持偏置校正")

        validity = document.get("validity")
        if not isinstance(validity, dict):
            raise ValueError("缺少validity配置")
        tolerance = self._finite_number(
            validity.get("endpoint_tolerance_v"), "endpoint_tolerance_v"
        )
        if tolerance < 0:
            raise ValueError("端点容差不能为负数")
        if validity.get("out_of_range") != "reject_group_and_fallback_raw":
            raise ValueError("不支持的校准越界策略")

        curves = document.get("curves")
        if not isinstance(curves, list) or len(curves) != 3:
            raise ValueError("校准配置必须包含三个通道传递曲线")
        for channel, curve in enumerate(curves):
            if not isinstance(curve, dict):
                raise ValueError(f"curves[{channel}]必须是对象")
            if curve.get("channel") != EXPECTED_CHANNEL_ORDER[channel]:
                raise ValueError("传递曲线顺序必须是ADC0、ADC1、ADC2")
            nodes = curve.get("nodes")
            if not isinstance(nodes, list) or len(nodes) < 2:
                raise ValueError(f"ADC{channel}传递曲线至少需要两个节点")
            previous_raw = None
            previous_reference = None
            for index, node in enumerate(nodes):
                if not isinstance(node, dict):
                    raise ValueError(f"ADC{channel}节点{index}必须是对象")
                raw = self._finite_number(
                    node.get("raw_voltage_v"),
                    f"ADC{channel}节点{index}原始电压",
                )
                reference = self._finite_number(
                    node.get("reference_voltage_v"),
                    f"ADC{channel}节点{index}参考电压",
                )
                sample_count = node.get("steady_sample_count")
                if (
                    isinstance(sample_count, bool)
                    or not isinstance(sample_count, int)
                    or sample_count <= 0
                ):
                    raise ValueError(f"ADC{channel}节点{index}样本数必须是正整数")
                if raw <= 0 or reference <= 0:
                    raise ValueError("ADC传递曲线节点电压必须大于0")
                if previous_raw is not None and raw <= previous_raw:
                    raise ValueError(f"ADC{channel}原始电压节点必须严格递增")
                if previous_reference is not None and reference < previous_reference:
                    raise ValueError(f"ADC{channel}参考电压节点不能回折")
                previous_raw = raw
                previous_reference = reference

            declared_min = self._finite_number(
                curve.get("input_min_v"), f"ADC{channel} input_min_v"
            )
            declared_max = self._finite_number(
                curve.get("input_max_v"), f"ADC{channel} input_max_v"
            )
            if declared_min != float(nodes[0]["raw_voltage_v"]):
                raise ValueError(f"ADC{channel}输入下限与首节点不一致")
            if declared_max != float(nodes[-1]["raw_voltage_v"]):
                raise ValueError(f"ADC{channel}输入上限与末节点不一致")

    def _interpolate_channel(self, channel_id, raw_voltage_v):
        bounded_raw_voltage_v = self._finite_number(raw_voltage_v, f"ADC{channel_id}电压")
        raw_voltage_nodes_v = self.raw_nodes[channel_id]
        minimum_raw_voltage_v, maximum_raw_voltage_v = self.input_ranges[channel_id]
        if bounded_raw_voltage_v < minimum_raw_voltage_v - self.endpoint_tolerance_v:
            raise ValueError(
                f"ADC{channel_id}电压{bounded_raw_voltage_v:.6f} V低于校准范围{minimum_raw_voltage_v:.6f} V"
            )
        if bounded_raw_voltage_v > maximum_raw_voltage_v + self.endpoint_tolerance_v:
            raise ValueError(
                f"ADC{channel_id}电压{bounded_raw_voltage_v:.6f} V高于校准范围{maximum_raw_voltage_v:.6f} V"
            )
        bounded_raw_voltage_v = min(maximum_raw_voltage_v, max(minimum_raw_voltage_v, bounded_raw_voltage_v))
        right = bisect.bisect_left(raw_voltage_nodes_v, bounded_raw_voltage_v)
        if right == 0:
            return self.curves[channel_id][0]["reference_voltage_v"]
        if right == len(raw_voltage_nodes_v):
            return self.curves[channel_id][-1]["reference_voltage_v"]
        if raw_voltage_nodes_v[right] == bounded_raw_voltage_v:
            return self.curves[channel_id][right]["reference_voltage_v"]
        left = right - 1
        fraction = (bounded_raw_voltage_v - raw_voltage_nodes_v[left]) / (
            raw_voltage_nodes_v[right] - raw_voltage_nodes_v[left]
        )
        left_reference_voltage_v = self.curves[channel_id][left]["reference_voltage_v"]
        right_reference_voltage_v = self.curves[channel_id][right]["reference_voltage_v"]
        return left_reference_voltage_v + fraction * (right_reference_voltage_v - left_reference_voltage_v)

    def apply(self, adc_voltages):
        """仅根据三个通道自身ADC电压进行校准，不接收DAC参数。"""
        if not isinstance(adc_voltages, (list, tuple)) or len(adc_voltages) != 3:
            raise ValueError("ADC电压必须按ADC0、ADC1、ADC2提供三个值")
        raw_voltages_v = [
            self._finite_number(value, f"ADC{channel}电压")
            for channel, value in enumerate(adc_voltages)
        ]
        calibrated_voltages_v = [
            self._interpolate_channel(channel, value)
            for channel, value in enumerate(raw_voltages_v)
        ]
        effective_gains = [
            calibrated_voltage_v / raw_voltage_v if raw_voltage_v != 0 else 1.0
            for raw_voltage_v, calibrated_voltage_v in zip(raw_voltages_v, calibrated_voltages_v)
        ]
        return calibrated_voltages_v, effective_gains

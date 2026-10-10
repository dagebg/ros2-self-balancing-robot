"""Tests for tools/gen_config.py."""

import gen_config
import pytest


def test_constant_names_are_camel_case_with_k_prefix():
    assert gen_config.to_constant_name("wheel_radius") == "kWheelRadius"
    assert gen_config.to_constant_name("uart_baud") == "kUartBaud"


@pytest.mark.parametrize(
    ("value", "expected"),
    [(7, ("std::int32_t", "7")), (0.5, ("float", "0.5F")), (150.0, ("float", "150.0F"))],
)
def test_cpp_literals(value, expected):
    assert gen_config.cpp_literal(value) == expected


@pytest.mark.parametrize("value", [True, "text", [1, 2]])
def test_unsupported_types_are_rejected(value):
    with pytest.raises(TypeError):
        gen_config.cpp_literal(value)


def _entry(value, measured=True):
    return {"value": value, "unit": "m", "measured": measured, "doc": "Doc."}


def test_placeholders_are_flagged_in_the_header():
    header = gen_config.render_header({"geometry": {"wheel_radius": _entry(0.03, False)}})
    assert "inline constexpr float kWheelRadius = 0.03F;" in header
    assert "PLACEHOLDER" in header
    assert "kAllParamsMeasured = false;" in header


def test_all_measured_header_has_no_placeholders():
    header = gen_config.render_header({"link": {"uart_baud": _entry(460800)}})
    assert "PLACEHOLDER" not in header
    assert "kAllParamsMeasured = true;" in header


def test_missing_keys_are_reported(tmp_path):
    params = tmp_path / "params.yaml"
    params.write_text("geometry:\n  wheel_radius:\n    value: 0.03\n", encoding="utf-8")
    with pytest.raises(ValueError, match="missing keys"):
        gen_config.load_params(params)


def test_duplicate_names_across_groups_are_rejected(tmp_path):
    entry = "    value: 1\n    unit: m\n    measured: true\n    doc: d\n"
    params = tmp_path / "params.yaml"
    params.write_text(f"a:\n  x:\n{entry}b:\n  x:\n{entry}", encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate"):
        gen_config.load_params(params)


def test_committed_header_is_up_to_date():
    assert gen_config.main(["--check"]) == 0

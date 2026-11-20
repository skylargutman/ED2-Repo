from serial_log import DataEcho


def test_data_lines_are_summarised_not_echoed():
    echo = DataEcho()
    assert echo.line("D 0 0.0 3.14 0.0 nan", now=0.0) is None
    for i in range(1, 500):
        assert echo.line(f"D {2 * i} 0.0 3.14 0.0 nan", now=i * 0.002) is None
    # the next line after a full second carries the count
    assert echo.line("D 1000 0.0 3.14 0.0 nan", now=1.0) == "(500 D lines in the last 1.0 s)"


def test_other_lines_are_echoed_unchanged():
    echo = DataEcho()
    assert echo.line(">> BALANCE", now=0.0) == ">> BALANCE"
    assert echo.line("run end stop", now=0.1) == "run end stop"


def test_summary_covers_a_quiet_gap():
    echo = DataEcho()
    echo.line("D 0 0 3.14 0 nan", now=0.0)
    assert echo.line(">> BALANCE", now=5.0) == ">> BALANCE"
    assert echo.line("D 2 0 3.14 0 nan", now=5.0) == "(1 D lines in the last 5.0 s)"
    assert echo.line("D 4 0 3.14 0 nan", now=5.5) is None

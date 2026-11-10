"""sim's questions (pick.choose) with typed answers, and what the shell gets, without ROS."""
import shlex
import subprocess

import pytest

from squad_gazebo import cars, pick
from test_cars import REGISTRY, fake_home, registry, share, write


def run(answers, current='', reg=None):
    typed, shown = list(answers), []

    def ask(prompt):
        shown.append(prompt)
        if not typed:
            raise EOFError
        return typed.pop(0)

    values = pick.choose(reg or registry(), current, ask, shown.append, share=share)
    return values, '\n'.join(shown)


def picked(values):
    return values['SQ_CAR'], values['SQ_WORLD'], values['SQ_HEADLESS'], values['SQ_AGENT']


def test_enter_everywhere_starts_picar_in_its_room_with_window_and_agent(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    values, shown = run(['', ''])
    assert picked(values) == ('picar', 'room', 'false', 'true')
    assert values['SQ_AGENT_RUN'] == 'python3 agent.py' and values['SQ_AGENT_ASK'] == ''
    assert values['SQ_AGENT_BEFORE'] == 'ollama stop qwen3-vl:4b-instruct'
    assert values['SQ_DOMAIN'] == 51
    assert '1) picar' in shown and '2) mini' in shown
    assert "pccar   (not ready: model: package 'pccar_description' is not built" in shown
    assert 'Gazebo window?' not in shown and 'agent too?' not in shown


def test_mini_by_number_gets_its_own_world(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    values, shown = run(['2', ''])
    assert picked(values) == ('mini', 'minirobot_room', 'false', 'true')
    assert values['SQ_AGENT_ASK'] == 'python3 minirobot_sim/ask.py'
    assert values['SQ_AGENT_ENV'].endswith('MiniRobotAI/.env')
    assert "minirobot_room   (mini's own)" in shown


def test_names_and_another_world(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    values, _ = run(['mini', 'room'])
    assert picked(values) == ('mini', 'room', 'false', 'true')


def test_not_ready_car_and_typos_are_asked_again(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    values, shown = run(['3', 'picra', '9', '1', ''])
    assert picked(values)[:2] == ('picar', 'room')
    assert "pccar cannot start: model: package 'pccar_description'" in shown
    assert '"picra" is not on the list.' in shown and '"9" is not on the list.' in shown


def test_mini_not_offered_when_its_folder_is_missing(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch, files=[])
    values, shown = run(['2', '1', ''])
    assert picked(values)[:2] == ('picar', 'room')
    assert 'mini cannot start: model:' in shown
    assert 'minirobot_room   (not ready:' in shown


def test_car_whose_agent_cannot_start_is_not_ready(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch, mini_env=False)
    values, shown = run(['mini', '', ''])
    assert picked(values)[:2] == ('picar', 'room')
    assert 'mini   (not ready: agent: ' in shown and '.env is missing' in shown
    assert 'mini cannot start: agent: ' in shown


def test_car_without_agent_starts_the_twin_only(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    reg = registry()
    del reg['cars']['mini']['agent']
    values, shown = run(['mini', ''], reg=reg)
    assert picked(values) == ('mini', 'minirobot_room', 'false', 'false')
    assert 'Agent: none set up for mini in cars.yaml, starting the twin only' in shown


def test_tab_already_set_up_only_asks_the_world(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    values, shown = run([''], current='mini')
    assert picked(values) == ('mini', 'minirobot_room', 'false', 'true')
    assert 'Which car?' not in shown and 'Car: mini' in shown


def test_tab_set_up_for_a_car_that_cannot_start(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    with pytest.raises(cars.CarError, match='pccar cannot start'):
        run([], current='pccar')


def test_ctrl_d_cancels(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    with pytest.raises(EOFError):
        run(['2'])


def test_shell_output_survives_eval(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    values = pick.info(registry(), 'mini')
    values['SQ_AGENT_PROBLEM'] = "it's \"odd\" $HOME `x`"
    script = pick.as_shell(values) + '\nprintf "%s|%s|%s" "$SQ_AGENT_ASK" "$SQ_DOMAIN" "$SQ_AGENT_PROBLEM"'
    out = subprocess.run(['bash', '-c', script], capture_output=True, text=True).stdout
    assert out == "python3 minirobot_sim/ask.py|53|it's \"odd\" $HOME `x`"


def test_info_for_a_car_without_agent(tmp_path, monkeypatch):
    fake_home(tmp_path, monkeypatch)
    values = pick.info(registry(), 'pccar')
    assert values['SQ_AGENT_RUN'] == '' and values['SQ_AGENT_PROBLEM'] == ''


def test_main_info_and_errors(tmp_path, monkeypatch, capsys):
    fake_home(tmp_path, monkeypatch)
    assert pick.main(['info', 'mini', '--cars', str(REGISTRY)]) == 0
    out = dict(line.split('=', 1) for line in capsys.readouterr().out.splitlines())
    assert shlex.split(out['SQ_CAR']) == ['mini'] and out['SQ_DOMAIN'] == '53'
    assert pick.main(['info', 'nope', '--cars', str(REGISTRY)]) == 1
    assert "Cannot start: unknown car 'nope'" in capsys.readouterr().err
    assert pick.main(['info']) == 2
    bad = write(tmp_path, 'cars: {x: {folder: "~/picar_ws"}}')
    assert pick.main(['info', 'x', '--cars', str(bad)]) == 1
    assert "needs 'domain:'" in capsys.readouterr().err
"""What the `sim` and `car` shell commands (squad.bash) need from cars.yaml.

    python3 -m squad_gazebo.pick ask [CAR]    # sim's questions; CAR: the car this tab is set up for
    python3 -m squad_gazebo.pick info CAR     # no questions, just CAR's settings
    ... --cars FILE                           # another cars.yaml than the installed one

Questions go to the terminal (stderr). The answer is printed on stdout as shell
assignments (SQ_CAR=..., SQ_WORLD=..., SQ_AGENT_RUN=...) for the shell to eval.
Cars, worlds and agents that cannot start are shown with the reason.
"""
import os
import shlex
import sys

from squad_gazebo import cars


def car_problems(registry, share):
    """car -> '' if its twin and its agent (if it has one) can start, else the reason."""
    problems = {}
    for name in registry['cars']:
        try:
            cars.resolve(registry, name, share=share)
            cars.tab(registry, name)
            cars.agent(registry, name)
            problems[name] = ''
        except cars.CarError as error:
            problems[name] = str(error).replace(f"car '{name}', ", '', 1)
    return problems


def world_problems(registry, share):
    problems = {}
    for name, spec in (registry.get('worlds') or {}).items():
        try:
            cars.file_in(spec, f"world '{name}'", share)
            problems[name] = ''
        except cars.CarError as error:
            problems[name] = str(error).replace(f"world '{name}': ", '', 1)
    return problems


def menu(title, problems, default, ask, say, notes=None):
    """A numbered list; the answer is a number, a name, or Enter for the default."""
    names = list(problems)
    say(title)
    for i, name in enumerate(names, 1):
        note = f'not ready: {problems[name]}' if problems[name] else (notes or {}).get(name, '')
        say(f'  {i}) {name}' + (f'   ({note})' if note else ''))
    while True:
        answer = ask(f'Number or name [{default}]: ').strip()
        if not answer:
            choice = default
        elif answer.isdigit() and 1 <= int(answer) <= len(names):
            choice = names[int(answer) - 1]
        elif answer in problems:
            choice = answer
        else:
            say(f'  "{answer}" is not on the list.')
            continue
        if problems[choice]:
            say(f'  {choice} cannot start: {problems[choice]}')
            continue
        return choice


def info(registry, car):
    """The car's tab and agent settings as SQ_* values. The agent's problem, if any, is
    in SQ_AGENT_PROBLEM (the tab still works without an agent)."""
    tab = cars.tab(registry, car)
    values = {'SQ_CAR': car, 'SQ_FOLDER': tab.folder, 'SQ_DOMAIN': tab.domain,
              'SQ_VENV': tab.venv}
    try:
        agent, problem = cars.agent(registry, car), ''
    except cars.CarError as error:
        agent, problem = None, str(error)
    values.update({
        'SQ_AGENT_DIR': agent.folder if agent else '',
        'SQ_AGENT_RUN': agent.run if agent else '',
        'SQ_AGENT_ENV': agent.env_file if agent else '',
        'SQ_AGENT_ASK': agent.ask if agent else '',
        'SQ_AGENT_BEFORE': agent.before if agent else '',
        'SQ_AGENT_WAIT': agent.wait_for if agent else '',
        'SQ_AGENT_PROBLEM': problem,
    })
    return values


def choose(registry, current, ask, say, share=cars.ros_share):
    """sim's questions: the car and the world. The Gazebo window and the agent always start.
    Returns the SQ_* values, with SQ_WORLD, SQ_HEADLESS and SQ_AGENT."""
    car_issues = car_problems(registry, share)
    ready = [name for name, problem in car_issues.items() if not problem]
    if current:
        if current not in car_issues:
            raise cars.CarError(f"this tab's car '{current}' is not in cars.yaml")
        if car_issues[current]:
            raise cars.CarError(f'{current} cannot start: {car_issues[current]}')
        car = current
        say(f'Car: {car} (this tab is set up for it; open a new tab for another car)')
    else:
        if not ready:
            raise cars.CarError('no car in cars.yaml can start: ' + '; '.join(
                f'{name}: {problem}' for name, problem in car_issues.items()))
        car = menu('Which car?', car_issues, ready[0], ask, say)

    world_issues = world_problems(registry, share)
    own = registry['cars'][car].get('world', '')
    default = own if own in world_issues and not world_issues[own] else next(
        (name for name, problem in world_issues.items() if not problem), '')
    if not default:
        raise cars.CarError('no world in cars.yaml can be loaded')
    world = menu('Which world?', world_issues, default, ask, say,
                 notes={own: f"{car}'s own"} if own else None)

    values = info(registry, car)
    if not values['SQ_AGENT_RUN']:
        say(f'Agent: none set up for {car} in cars.yaml, starting the twin only')
    values.update({'SQ_WORLD': world, 'SQ_HEADLESS': 'false',
                   'SQ_AGENT': 'true' if values['SQ_AGENT_RUN'] else 'false'})
    return values


def as_shell(values):
    return '\n'.join(f'{key}={shlex.quote(str(value))}' for key, value in values.items())


def terminal_ask(prompt):
    sys.stderr.write(prompt)
    sys.stderr.flush()
    line = sys.stdin.readline()
    if not line:
        raise EOFError
    return line


def main(argv=None):
    def say(text):
        print(text, file=sys.stderr)

    args = list(sys.argv[1:] if argv is None else argv)
    registry_file = ''
    if '--cars' in args:
        i = args.index('--cars')
        registry_file = args[i + 1] if i + 1 < len(args) else ''
        del args[i:i + 2]
    if not args or args[0] not in ('ask', 'info') or (args[0] == 'info' and len(args) != 2):
        say('usage: python3 -m squad_gazebo.pick ask [CAR] | info CAR  [--cars FILE]')
        return 2
    try:
        registry = cars.load(registry_file or os.path.join(
            cars.ros_share('squad_gazebo'), 'config', 'cars.yaml'))
        if args[0] == 'info':
            values = info(registry, args[1])
        else:
            values = choose(registry, args[1] if len(args) > 1 else '', terminal_ask, say)
    except cars.CarError as error:
        say(f'Cannot start: {error}')
        return 1
    except (EOFError, KeyboardInterrupt):
        say('\nCancelled.')
        return 1
    print(as_shell(values))
    return 0


if __name__ == '__main__':
    sys.exit(main())
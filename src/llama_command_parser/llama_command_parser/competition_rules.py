"""Solve the published container-demand templates before releasing robot tasks."""
import json
import re
import uuid


RULES = {
    1: 'x代表红色，y代表蓝色；数量大于等于3去A区，否则去C区',
    2: '数量大于等于3代表红色，否则代表蓝色；x去B区，y去A区',
    3: '较大的数值代表蓝色，较小的代表红色；蓝色去B区，红色去C区',
}
NUMBER = r'(?:\d+|[零〇一二两三四五六七八九十百千]+)'
CONTAINERS = r'(?:衣柜|书箱|礼品盒|礼盒|玩具箱|箱|盒)'
UNITS = r'(?:件|条|本|个|颗|粒)'


def extract_mapping(problem):
    """Separate a numbered or fully written official mapping from the question."""
    match = re.search(r'(?:映射规则|规则编号)\s*[:：]\s*([123])\s*[。.]?\s*$', problem)
    if match:
        return problem[:match.start()].strip(), int(match[1])
    start = re.search(
        r'映射规则\s*[:：]|[xX]\s*代表|(?:数量|数值)\s*(?:大于等于|≥|>=)|较大(?:的数值|的数|值)',
        problem)
    if not start:
        return problem, None
    suffix = problem[start.start():]
    suffix = re.sub(r'^映射规则\s*[:：]\s*', '', suffix)
    compact = re.sub(r'[\s，,；;。\.、]', '', suffix).lower()
    compact = compact.replace('>=', '大于等于').replace('≥', '大于等于')
    threshold = r'(?:数量|数值)大于等于(?:3|三)(?:的)?'
    otherwise = r'(?:其他颜色|其他|其余|否则)'
    go = r'(?:去|到)'
    patterns = {
        1: rf'x代表红色y代表蓝色{threshold}{go}a区{otherwise}{go}c区',
        2: rf'{threshold}代表红色{otherwise}代表蓝色x{go}b区y{go}a区',
        3: rf'较大(?:的数值|的数|值)代表蓝色较小(?:的数值|的数|值|的)?代表红色蓝色{go}b区红色{go}c区',
    }
    for rule, pattern in patterns.items():
        if re.fullmatch(pattern, compact):
            return problem[:start.start()].strip(), rule
    raise ValueError('未识别完整映射规则，请核对三条正式规则，或在题末填写“映射规则：1/2/3”')


def number(text):
    if text.isdigit():
        value = int(text)
    else:
        digits = dict(zip('零〇一二两三四五六七八九', (0, 0, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9)))
        value, pending = 0, 0
        for char in text:
            if char in digits:
                pending = digits[char]
            elif char in '十百千':
                value += (pending or 1) * {'十': 10, '百': 100, '千': 1000}[char]
                pending = 0
            else:
                raise ValueError(f'不支持的数字：{text}')
        value += pending
    if not 0 <= value <= 10000:
        raise ValueError('题目中的数量超出支持范围')
    return value


def parse_input(text, default_rule=1):
    """Accept a plain official problem, or explicit {problem, rule} JSON."""
    if not isinstance(text, str) or not text.strip() or len(text) > 12000:
        raise ValueError('请输入完整题目，长度不超过12000字符')
    problem, rule = text.strip(), default_rule
    explicit_rule = False
    if problem.startswith('{'):
        payload = json.loads(problem)
        problem, rule = payload['problem'], payload.get('rule', default_rule)
        explicit_rule = 'rule' in payload
    if type(rule) is not int or rule not in RULES:
        raise ValueError('映射规则必须为整数1、2或3')
    if not isinstance(problem, str) or not problem.strip():
        raise ValueError('problem必须是完整题目文字')
    problem, written_rule = extract_mapping(problem)
    if written_rule is not None:
        if explicit_rule and written_rule != rule:
            raise ValueError('rule编号与题末映射规则不一致，请核对')
        rule = written_rule
    if not problem.strip():
        raise ValueError('请在映射规则之前填写完整题目')
    return problem.strip(), rule


def solve_container_problem(problem):
    """Interpret container demands using the question's explicit x/y definitions.

    Reject unrecognized clauses rather than silently guessing a demand. The
    smallest whole container count is ceil(total demand / capacity).
    """
    text = re.sub(r'\s+', '', problem).replace('t恤', 'T恤')
    if re.search(rf'(?:[-负]{NUMBER}|\d+\.\d+){UNITS}', text):
        raise ValueError('需求必须是非负整数，请核对题目')
    capacity_match = re.search(
        rf'每(?:个|箱)?({CONTAINERS})(?:里|中)?(?:有|装|可装|能装)({NUMBER}){UNITS}', text)
    if not capacity_match:
        raise ValueError('未识别容器容量，请保留“每个衣柜/书箱/礼品盒/玩具箱有N件/本/个物品”等完整描述')
    capacity = number(capacity_match[2])
    if capacity == 0:
        raise ValueError('每柜/箱容量必须大于零')
    definitions = {}
    definition_text = text.split('设', 1)[-1]
    # In “故事书箱数”, the item is 故事书 and the suffix is 箱.
    # In “软糖礼盒数量”, 礼盒 is the suffix; do not include 礼 in the item.
    definition_container = ('箱' if capacity_match[1] in ('箱', '书箱') else
                            r'(?:礼品盒|礼盒|盒)' if capacity_match[1] in ('礼品盒', '礼盒', '盒') else
                            '玩具箱' if capacity_match[1] == '玩具箱' else
                            '衣柜')
    for match in re.finditer(
            rf'(?:^|[，,；;。])(?:设)?([\u4e00-\u9fffA-Za-z]{{1,16}}?)'
            rf'({definition_container})(?:的)?(?:数量|数)?(?:为|是|=)([xXyY])(?=$|[，,；;。、])',
            definition_text):
        item, container, var = match[1], match[2], match[3].lower()
        family = lambda noun: {'礼盒': '盒', '礼品盒': '盒', '书箱': '箱'}.get(noun, noun)
        if family(container) != family(capacity_match[1]):
            raise ValueError('x、y定义的容器与题首容量描述不一致')
        if var in definitions and definitions[var] != item:
            raise ValueError(f'{var}的定义不唯一')
        definitions[var] = item
    if set(definitions) != {'x', 'y'} or definitions['x'] == definitions['y']:
        raise ValueError('请包含题末x、y分别代表哪类衣柜/书箱/礼盒/玩具箱的定义')
    aliases = '|'.join(map(re.escape, sorted(definitions.values(), key=len, reverse=True)))
    demand_pattern = rf'({NUMBER}){UNITS}({aliases})'
    body = text.split('设', 1)[0]
    clauses = [clause for clause in re.split(r'[;；。\n]', body) if clause]
    records, pending = {}, []
    for clause in clauses:
        # “小车” is a declared item, not a second person in “小杰想玩2个小车”.
        # Mask item names in the header, quantities and replacements before
        # finding the subject/reference names; keep the original demand text.
        people_text = re.sub(rf'(?:{aliases})', '', clause)
        names = list(dict.fromkeys(re.findall(r'小[\u4e00-\u9fff]', people_text)))
        if names:
            pending.append((clause, names))
        elif re.search(r'想要|想玩|需要|需求|少|多', clause):
            raise ValueError(f'未识别需求人物：{clause}')
    if not pending:
        raise ValueError('未识别人物需求，不能猜测x、y')
    while pending:
        progress, deferred = False, []
        for clause, names in pending:
            for quantity in re.finditer(rf'{NUMBER}{UNITS}', clause):
                if not re.match(rf'(?:{aliases})', clause[quantity.end():]):
                    raise ValueError(f'数量后未识别物品种类：{clause}')
            same = '一样' in clause or '相同' in clause
            if same:
                if len(names) < 2:
                    raise ValueError(f'未识别“一样”的参照人物：{clause}')
                subjects, reference = names[:-1], names[-1]
                if reference not in records:
                    deferred.append((clause, names))
                    continue
                counts = records[reference]['demand'].copy()
                replacement = re.search(rf'换成({aliases})', clause)
                if replacement:
                    if sum(v > 0 for v in counts.values()) != 1:
                        raise ValueError('换种类的参照需求包含多种物品，请确认题意')
                    total = sum(counts.values())
                    counts = {item: total if item == replacement[1] else 0
                              for item in definitions.values()}
                adjustments = re.findall(rf'(少|多)({NUMBER}){UNITS}({aliases})', clause)
                if len(re.findall(demand_pattern, clause)) != len(adjustments):
                    raise ValueError(f'参照需求中的新增数量不明确：{clause}')
                if ('少' in clause or '多' in clause) and not adjustments:
                    raise ValueError(f'未识别增减数量：{clause}')
                for direction, amount, item in adjustments:
                    counts[item] += number(amount) * (-1 if direction == '少' else 1)
            else:
                subjects = names
                found = re.findall(demand_pattern, clause)
                if not found or not re.search(r'想要|想玩|需要|需|都要|要', clause):
                    raise ValueError(f'未识别完整需求：{clause}')
                counts = {item: 0 for item in definitions.values()}
                for amount, item in found:
                    counts[item] += number(amount)
            factors = re.findall(rf'({NUMBER}|双)倍', clause)
            if factors:
                if len(subjects) != 1 or len(set(factors)) != 1:
                    raise ValueError('倍数需求的适用人物不明确，请确认题意')
                factor = 2 if factors[0] == '双' else number(factors[0])
                counts = {item: value * factor for item, value in counts.items()}
            if any(value < 0 for value in counts.values()):
                raise ValueError('增减后的物品需求为负数，请确认题目')
            for name in subjects:
                if name in records:
                    raise ValueError(f'{name}重复定义需求，请核对题目')
                records[name] = {'name': name, 'demand': counts.copy(), 'source': clause}
            progress = True
        if not progress:
            raise ValueError('参照人物不存在或需求循环依赖')
        pending = deferred
    totals = {item: sum(r['demand'][item] for r in records.values())
              for item in definitions.values()}
    answers = {var: (totals[item] + capacity - 1) // capacity
               for var, item in definitions.items()}
    return dict(answers=answers, definitions=definitions, capacity=capacity,
                totals=totals, demands=list(records.values()),
                calculation='各类需求合计后，按每柜/箱容量向上取整；需与裁判确认此题意')


def map_tasks(x, y, rule):
    if type(x) is not int or type(y) is not int or min(x, y) < 0:
        raise ValueError('x、y必须为非负整数')
    if type(rule) is not int or rule not in RULES:
        raise ValueError('映射规则必须为1、2、3')
    if rule == 3 and x == y:
        raise ValueError('规则3未定义x=y的处理方式，请向裁判确认')
    tasks = []
    for variable, value in [('x', x), ('y', y)]:
        if not value:
            continue
        if rule == 1:
            color = 'red' if variable == 'x' else 'blue'
            zone = 'A' if value >= 3 else 'C'
        elif rule == 2:
            color = 'red' if value >= 3 else 'blue'
            zone = 'B' if variable == 'x' else 'A'
        else:
            color = 'blue' if value == max(x, y) else 'red'
            zone = 'B' if color == 'blue' else 'C'
        tasks.append(dict(color=color, num=value, to=zone))
    if not tasks:
        raise ValueError('映射后的任务为空')
    return tasks


def validate_inventory(tasks, inventory):
    for color in ('red', 'blue'):
        required = sum(t['num'] for t in tasks if t['color'] == color)
        available = inventory.get(color)
        if type(available) is not int or available < required:
            raise ValueError(f'{color}需要{required}件，当前可用{available}件')


class CompetitionSession:
    """A preview never becomes /chat until its exact ID is confirmed once."""
    def __init__(self):
        self.pending = None

    def prepare(self, problem, rule, inventory):
        self.pending = None  # Failed replacement must also invalidate the old preview.
        solution = solve_container_problem(problem)
        tasks = map_tasks(solution['answers']['x'], solution['answers']['y'], rule)
        validate_inventory(tasks, inventory)
        self.pending = dict(preview_id=uuid.uuid4().hex[:12], problem=problem,
                            rule=rule, rule_text=RULES[rule], solution=solution,
                            tasks=tasks, state='awaiting_confirmation')
        return self.pending

    def confirm(self, preview_id, confirmed, ready, inventory):
        if type(confirmed) is not bool or not confirmed:
            raise ValueError('必须明确confirm=true，表示已与裁判核对')
        if self.pending is None or preview_id != self.pending['preview_id']:
            raise ValueError('预览编号失效或任务已发出，请重新准备题目')
        if not ready:
            raise ValueError('主控未准备好或仍有任务，请先等待当前任务结束')
        validate_inventory(self.pending['tasks'], inventory)
        tasks = self.pending['tasks']
        self.pending = None
        return tasks

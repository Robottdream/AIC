"""Official sample arithmetic, mapping and release guards; no robot commands."""
import ast
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src/llama_command_parser'))
from llama_command_parser.competition_rules import (
    CompetitionSession, map_tasks, number, parse_input, solve_container_problem,
    validate_inventory,
)

WARDROBE = ('每个衣柜有5件衣物，有T恤、裤子两种衣柜；'
            '小美想要1件T恤；小帅想要1条裤子、10件T恤；'
            '小丽说和小美一样；小娜要1条裤子；'
            '小玉也和小美一样但是换成裤子；小杰想要5件T恤、1条裤子。'
            '设T恤衣柜数量为x，裤子衣柜数量为y，计算x、y。')
BOOKS = ('每个书箱有5本书籍，有故事书、科普书两种书箱。'
         '小文想要1本故事书，5本科普书；'
         '小雅想要1本故事书，他是双胞胎所以需要双倍的书；'
         '小静和小文需求一样，但少一本故事书；'
         '小豪想要5本科普书；小智想要1本故事书，5本科普书。'
         '设故事书箱数为x，科普书箱数为y，计算x、y。')
INVENTORY = dict(red=5, blue=5)
GIFTS = ('每个礼品盒有5个糖果;有软糖、硬糖两种礼品盒。'
         '小红想要1个软糖;小刚想要1个硬糖,10个软糖;小明说和小红一样;'
         '小兰要1个硬糖;小玉也和小红一样但是换成硬糖;小华想要5个软糖，1个硬糖。'
         '设软糖礼盒数量为x,硬糖礼盒数量为y,计算x、y。')
GIFT_RULE = 'x代表红色,y代表蓝色;数值大于等于3的去A区,其他颜色去C区'


class RulesTests(unittest.TestCase):
    def test_gift_boxes_with_full_written_mapping(self):
        problem, rule = parse_input(GIFTS + ' ' + GIFT_RULE)
        self.assertEqual((problem, rule), (GIFTS, 1))
        result = solve_container_problem(problem)
        self.assertEqual(result['definitions'], {'x': '软糖', 'y': '硬糖'})
        self.assertEqual(result['totals'], {'软糖': 17, '硬糖': 4})
        self.assertEqual(result['answers'], {'x': 4, 'y': 1})
        self.assertEqual(CompetitionSession().prepare(problem, rule, INVENTORY)['tasks'],
                         [dict(color='red', num=4, to='A'), dict(color='blue', num=1, to='C')])

    def test_declared_items_are_not_a_fixed_vocabulary(self):
        changed = GIFTS.replace('软糖', '巧克力').replace('硬糖', '饼干').replace('个', '颗')
        # Capacity still uses the container counter 个; contents use 颗.
        changed = changed.replace('每颗礼品盒', '每个礼品盒')
        self.assertEqual(solve_container_problem(changed)['totals'], {'巧克力': 17, '饼干': 4})
        changed = GIFTS.replace('10个软糖', '2个软糖')
        self.assertEqual(solve_container_problem(changed)['answers'], {'x': 2, 'y': 1})
        for bad in (GIFTS.replace('10个软糖', '10个水果'),
                    GIFTS.replace('10个软糖', '-1个软糖'),
                    GIFTS.replace('10个软糖', '0.5个软糖'),
                    GIFTS.replace('小兰要', '小红要')):
            with self.subTest(problem=bad), self.assertRaises(ValueError):
                solve_container_problem(bad)

    def test_full_mapping_rules_and_conflicts(self):
        from llama_command_parser.competition_rules import RULES
        for rule, written in RULES.items():
            for prefix in ('', '映射规则：'):
                self.assertEqual(parse_input(GIFTS + '\n' + prefix + written), (GIFTS, rule))
        self.assertEqual(parse_input(GIFTS + '\n映射规则：2。'), (GIFTS, 2))
        for written in (GIFT_RULE.replace('A区', 'B区'), GIFT_RULE + '，红色到B区',
                        '映射规则：4'):
            with self.subTest(mapping=written), self.assertRaises(ValueError):
                parse_input(GIFTS + written)
        with self.assertRaises(ValueError):
            parse_input(json.dumps(dict(problem=GIFTS + GIFT_RULE, rule=2)))

    def test_official_wardrobe(self):
        result = solve_container_problem(WARDROBE)
        self.assertEqual(result['totals'], {'T恤': 17, '裤子': 4})
        self.assertEqual(result['answers'], {'x': 4, 'y': 1})
        self.assertEqual(len(result['demands']), 6)

    def test_official_books(self):
        result = solve_container_problem(BOOKS)
        self.assertEqual(result['totals'], {'故事书': 4, '科普书': 20})
        self.assertEqual(result['answers'], {'x': 1, 'y': 4})

    def test_definition_order_and_chinese_numbers(self):
        problem = WARDROBE.replace('10件', '十件').replace('为x', '为Z')
        problem = problem.replace('为y', '为x').replace('为Z', '为y')
        self.assertEqual(solve_container_problem(problem)['answers'], {'y': 4, 'x': 1})
        self.assertEqual(number('一百零二'), 102)

    def test_shared_people_and_forward_reference(self):
        problem = ('每个衣柜有5件衣物；小丽和小娜与小美需求一样；'
                   '小美想要2件T恤、1条裤子。设T恤衣柜数量为x，裤子衣柜数量为y。')
        self.assertEqual(solve_container_problem(problem)['totals'], {'T恤': 6, '裤子': 3})

    def test_unsupported_and_invalid_fail_closed(self):
        for problem in (WARDROBE.replace('小丽说和小美一样', '小丽任意选择衣服'),
                        WARDROBE.replace('有5件', '有0件'),
                        WARDROBE.replace('换成裤子', '少两件T恤'),
                        WARDROBE.replace('想要1件T恤', '想要-1件T恤'),
                        WARDROBE.replace('想要1件T恤', '想要0.5件T恤'),
                        WARDROBE.replace('小丽说和小美一样', '小丽和小美一样，还要2条裤子'),
                        WARDROBE.replace('10件T恤', '10件毛衣'),
                        WARDROBE.replace('T恤衣柜数量', 'T恤数量'),
                        WARDROBE.replace('为x', '为z'),
                        WARDROBE.replace('小娜要', '小美要')):
            with self.subTest(problem=problem), self.assertRaises(ValueError):
                solve_container_problem(problem)
        cyclic = ('每个书箱有5本书；小美和小丽一样；小丽和小美一样。'
                  '设故事书箱数为x，科普书箱数为y。')
        with self.assertRaises(ValueError):
            solve_container_problem(cyclic)

    def test_input_formats(self):
        self.assertEqual(parse_input(WARDROBE + '\n映射规则：2'), (WARDROBE, 2))
        self.assertEqual(parse_input(json.dumps({'problem': BOOKS, 'rule': 3})), (BOOKS, 3))
        for rule in ('1', True, 4):
            with self.assertRaises(ValueError):
                parse_input(json.dumps(dict(problem=BOOKS, rule=rule)))

    def test_three_rules_and_boundary(self):
        self.assertEqual(map_tasks(3, 2, 1), [dict(color='red', num=3, to='A'),
                                            dict(color='blue', num=2, to='C')])
        self.assertEqual(map_tasks(2, 3, 2), [dict(color='blue', num=2, to='B'),
                                            dict(color='red', num=3, to='A')])
        self.assertEqual(map_tasks(1, 4, 3), [dict(color='red', num=1, to='C'),
                                            dict(color='blue', num=4, to='B')])
        self.assertEqual(map_tasks(0, 3, 1), [dict(color='blue', num=3, to='A')])
        with self.assertRaises(ValueError):
            map_tasks(2, 2, 3)

    def test_inventory_aggregates_same_color(self):
        with self.assertRaises(ValueError):
            validate_inventory(map_tasks(3, 3, 2), INVENTORY)

    def test_confirmation_once_and_matching_id(self):
        session = CompetitionSession()
        preview = session.prepare(WARDROBE, 1, INVENTORY)
        pid = preview['preview_id']
        for args in ((pid, False, True, INVENTORY), ('wrong', True, True, INVENTORY),
                     (pid, True, False, INVENTORY), (pid, True, True, dict(red=3, blue=5))):
            with self.assertRaises(ValueError):
                session.confirm(*args)
        self.assertEqual(session.confirm(pid, True, True, INVENTORY), map_tasks(4, 1, 1))
        with self.assertRaises(ValueError):
            session.confirm(pid, True, True, INVENTORY)

    def test_invalid_replacement_invalidates_old_preview(self):
        session = CompetitionSession()
        pid = session.prepare(WARDROBE, 1, INVENTORY)['preview_id']
        with self.assertRaises(ValueError):
            session.prepare('未知题目', 1, INVENTORY)
        with self.assertRaises(ValueError):
            session.confirm(pid, True, True, INVENTORY)


def parser_callbacks():
    path = ROOT / 'src/llama_command_parser/llama_command_parser/command_parser_node.py'
    tree = ast.parse(path.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    cls.bases = []
    cls.body = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                and n.name in {'publish_competition_status', 'fresh_controller_state',
                               'problem_callback', 'competition_start_callback'}]
    namespace = dict(json=json, time=time, parse_input=parse_input,
                     String=lambda **kw: NS(**kw))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), str(path), 'exec'), namespace)
    node = namespace[cls.name]()
    node.competition_session = CompetitionSession()
    node.competition_rule = 1
    node.controller_ready = dict(ready=True, inventory=INVENTORY)
    node.controller_ready_at = time.monotonic()
    node.competition_model_check = lambda preview: preview.update(llm_matches=True)
    node.latest_chat_msg = NS(data='')
    node.get_logger = lambda: NS(error=lambda *_: None, warn=lambda *_: None)
    for name in ('chat_pub', 'preview_pub', 'competition_status_pub'):
        messages = []
        setattr(node, name, NS(messages=messages, publish=lambda msg, out=messages: out.append(msg.data)))
    return node


class ReleaseTests(unittest.TestCase):
    def test_gift_preview_remains_confirmation_only(self):
        node = parser_callbacks()
        node.problem_callback(NS(data=GIFTS + '\n' + GIFT_RULE))
        self.assertEqual(node.chat_pub.messages, [])
        preview = json.loads(node.preview_pub.messages[-1])
        self.assertEqual(preview['solution']['answers'], {'x': 4, 'y': 1})
        self.assertEqual(preview['rule'], 1)
        self.assertEqual(preview['state'], 'awaiting_confirmation')

    def test_prepare_never_sends_task_and_confirmation_sends_once(self):
        node = parser_callbacks()
        node.problem_callback(NS(data=WARDROBE))
        self.assertEqual(node.chat_pub.messages, [])
        preview = json.loads(node.preview_pub.messages[-1])
        command = NS(data=json.dumps(dict(preview_id=preview['preview_id'], confirm=True)))
        node.competition_start_callback(command)
        node.competition_start_callback(command)
        self.assertEqual(len(node.chat_pub.messages), 1)
        self.assertEqual(json.loads(node.chat_pub.messages[0]), map_tasks(4, 1, 1))

    def test_stale_controller_and_bad_new_input(self):
        node = parser_callbacks()
        node.problem_callback(NS(data=WARDROBE))
        pid = node.competition_session.pending['preview_id']
        node.controller_ready_at -= 6
        node.competition_start_callback(NS(data=json.dumps(dict(preview_id=pid, confirm=True))))
        self.assertEqual(node.chat_pub.messages, [])
        node.controller_ready_at = time.monotonic()
        node.problem_callback(NS(data='{"rule":1}'))
        node.competition_start_callback(NS(data=json.dumps(dict(preview_id=pid, confirm=True))))
        self.assertEqual(node.chat_pub.messages, [])

    def test_model_failure_does_not_leave_startable_preview(self):
        node = parser_callbacks()
        def fail(_):
            raise RuntimeError('model unavailable')
        node.competition_model_check = fail
        node.problem_callback(NS(data=WARDROBE))
        self.assertIsNone(node.competition_session.pending)
        self.assertEqual(node.chat_pub.messages, [])


if __name__ == '__main__':
    unittest.main()

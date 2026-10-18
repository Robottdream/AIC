#!/usr/bin/env python3
"""Export the competition dashboard using Foxglove's existing panel settings."""
import copy
import itertools
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / 'foxglove_competition_layout.json'
PREFIX = '/foxglove_script/competition_dashboard'


def split(first, second, direction='column', percent=50):
    return dict(first=first, second=second, direction=direction, splitPercentage=percent)


def indicator(path, title, values, color, fallback, font=36):
    return dict(path=PREFIX+'.'+path, style='background', fontSize=font,
                fallbackColor=color, fallbackLabel=fallback,
                rules=[dict(operator='=', rawValue=str(value), label=label, color=color)
                       for value, label in values], foxglovePanelTitle=title)


def raw(path, title, font=12):
    return dict(diffEnabled=False, diffMethod='custom', diffTopicPath='',
                showFullMessageForDiff=False, topicPath=path, fontSize=font,
                foxglovePanelTitle=title)


def build():
    old = json.loads((ROOT/'demo1.json').read_text())['configById']
    panels = {}
    panels['Publish!questionInput'] = dict(
        buttonText='解析新题目',
        buttonTooltip='在 data 中粘贴完整新题目，并在末尾追加“映射规则：1”（或2、3）；此按钮仅生成预览。',
        advancedView=True, value=json.dumps(dict(data=''), ensure_ascii=False, indent=2),
        topicName='/competition/problem', datatype='std_msgs/msg/String',
        foxglovePanelTitle='题目与映射规则（粘贴当轮原文）')
    panels['RawMessages!questionReadback'] = raw(PREFIX+'.preparation', '原题 / 映射规则 / 解析状态', 12)
    for color, label, background in [('red','红色','#e96571'),('blue','蓝色','#4055ef')]:
        panels[f'Indicator!{color}Count'] = indicator(color+'_count', label+'数量',
            [(i,str(i)) for i in range(11)], background, '待解析',48)
        areas = [''.join(items) for n in range(1,4) for items in itertools.combinations('ABC',n)]
        panels[f'Indicator!{color}Area'] = indicator(color+'_areas', label+'放置区域',
            [(a,label+'到'+'、'.join(a)+'区') for a in areas]+[('-', '无'+label+'任务')],
            '#efb095' if color=='red' else '#89bfd8','等待解析',30)
    panels['Image!rgb'] = copy.deepcopy(old['Image!1p1fyg9'])
    panels['Image!rgb']['imageMode'] = dict(imageTopic='/camera/image_raw',
        calibrationTopic='/camera/camera_info',synchronize=False,rotation=0)
    panels['Image!rgb']['foxglovePanelTitle'] = '识别画面（彩色）'
    panels['Image!depth'] = copy.deepcopy(old['Image!48829cr'])
    panels['Image!depth']['foxglovePanelTitle'] = '识别画面（深度）'
    panels['3D!competitionMap'] = copy.deepcopy(old['3D!1juc55f'])
    panels['3D!competitionMap']['foxglovePanelTitle'] = '三维地图 / 小车 / 雷达'
    panels['Indicator!progress'] = indicator('piece_index','任务进度',
        [(0,'全部完成')]+[(i,f'第 {i} 个') for i in range(1,11)],'#bceacb','等待新任务',36)
    panels['Indicator!phase'] = indicator('phase','当前任务',list(enumerate([
        '等待任务','选择可达物块','前往资源点','正在抓取','前往放置区','正在放置',
        '全部完成','停止 / 待核验','等待核对并启动','解析失败'])), '#bceacb','等待状态',30)
    panels['Indicator!arm'] = indicator('arm','抓取 / 释放状态',list(enumerate([
        '未抓取','抓取中','已抓取','放置中','已释放','抓取重试','放置失败','修正放置位置'])),
        '#bceacb','等待状态',30)
    panels['Indicator!placed'] = indicator('placed','已确认放置',
        [(i,f'{i} 件') for i in range(11)],'#d6e5f0','等待确认',30)
    panels['RawMessages!preview'] = raw('/competition/preview','核对答案与区域 / 复制 preview_id',12)
    panels['Publish!startTask'] = dict(buttonText='核对后开始任务',
        buttonTooltip='从当前预览复制 preview_id，填入下方消息后确认；每个编号只能启动一次。',
        advancedView=True,
        value=json.dumps(dict(data=json.dumps(dict(preview_id='',confirm=True),ensure_ascii=False)),
                         ensure_ascii=False,indent=2),
        topicName='/competition/start',datatype='std_msgs/msg/String',foxglovePanelTitle='开始任务（填写本次预览编号）')
    counts = split(split('Indicator!redCount','Indicator!blueCount','row'),
                   split('Indicator!blueArea','Indicator!redArea','row'),'row',44)
    status = split(split('Indicator!progress','Indicator!phase'),
                   split('Indicator!arm','Indicator!placed'),'column',50)
    controls = split(status,split('RawMessages!preview','Publish!startTask','column',55),'column',47)
    visuals = split(split('Image!rgb','Image!depth','row'),'3D!competitionMap','column',43)
    layout = split(split('Publish!questionInput','RawMessages!questionReadback','row',69),
                   split(counts,split(visuals,controls,'row',72),'column',15),'column',24)
    result = dict(configById=panels, globalVariables={},
                  userNodes={'aic-competition-dashboard':dict(name='比赛结果与实际进度',
                      sourceCode=(ROOT/'tools/foxglove/competition_dashboard.ts').read_text())},
                  playbackConfig=dict(speed=1),layout=layout)
    OUTPUT.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
    return result


if __name__ == '__main__':
    result=build()
    print(f'{OUTPUT}: {len(result["configById"])} panels, 1 embedded user script')

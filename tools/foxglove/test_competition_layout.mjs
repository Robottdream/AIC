import { readFileSync } from 'node:fs';
import { stripTypeScriptTypes } from 'node:module';
import { test } from 'node:test';
import assert from 'node:assert/strict';
const root = new URL('../../', import.meta.url);
const layout = JSON.parse(readFileSync(new URL('foxglove_competition_layout.json', root)));
const source = layout.userNodes['aic-competition-dashboard'].sourceCode;
const javascript = stripTypeScriptTypes(source, { mode: 'strip' });
let serial = 0;
async function dashboard() {
  const m = await import('data:text/javascript;base64,' + Buffer.from(javascript).toString('base64') + '#' + (++serial));
  return (topic, value) => m.default({ topic, message: { data: typeof value === 'string' ? value : JSON.stringify(value) } });
}
const books = [{color:'red', num:1, to:'C'}, {color:'blue', num:4, to:'A'}];
const wardrobe = [{color:'red', num:4, to:'A'}, {color:'blue', num:1, to:'C'}];

test('export is complete, embeds the source and has no fixed sample task', () => {
  assert.equal(source, readFileSync(new URL('tools/foxglove/competition_dashboard.ts', root), 'utf8'));
  const ids = [];
  function walk(node) {
    if (typeof node === 'string') { ids.push(node); return; }
    assert.ok(['row','column'].includes(node.direction));
    assert.ok(node.splitPercentage > 0 && node.splitPercentage < 100);
    walk(node.first); walk(node.second);
  }
  walk(layout.layout);
  assert.equal(new Set(ids).size, ids.length);
  assert.deepEqual(ids.slice().sort(), Object.keys(layout.configById).sort());
  const input = layout.configById['Publish!questionInput'];
  assert.deepEqual(JSON.parse(input.value), { data:'' });
  assert.equal(input.topicName, '/competition/problem');
  const start = layout.configById['Publish!startTask'];
  assert.deepEqual(JSON.parse(JSON.parse(start.value).data), {preview_id:'', confirm:true});
  assert.equal(start.topicName, '/competition/start');
  assert.equal(javascript.includes('publish('), false); // Script transforms messages only.
});

test('different parsed tasks replace counts and destinations', async () => {
  const send = await dashboard();
  let out = send('/competition/preview',{problem:'书箱题', rule_text:'规则1', tasks:books, preview_id:'new1'});
  assert.deepEqual([out.red_count,out.blue_count,out.red_areas,out.blue_areas],[1,4,'C','A']);
  out = send('/competition/preview',{problem:'衣柜题', rule_text:'规则1', tasks:wardrobe, preview_id:'new2'});
  assert.deepEqual([out.red_count,out.blue_count,out.red_areas,out.blue_areas],[4,1,'A','C']);
  assert.equal(out.preview_id,'new2');
  assert.equal(out.problem,'衣柜题');
});

test('multiple destinations for the same color and absent colors remain accurate', async () => {
  const send = await dashboard();
  const out = send('/competition/preview',{tasks:[{color:'red',num:3,to:'B'},{color:'red',num:3,to:'A'}]});
  assert.deepEqual([out.red_count,out.blue_count,out.red_areas,out.blue_areas],[6,0,'AB','-']);
});

test('parsing failure clears stale preview results', async () => {
  const send = await dashboard();
  send('/competition/preview',{tasks:books,preview_id:'old'});
  const out = send('/competition/preview',{state:'error',error:'题目不完整'});
  assert.deepEqual([out.red_count,out.blue_count,out.red_areas,out.preview_id],[-1,-1,'','']);
  assert.equal(out.notice,'题目不完整');
});

test('progress follows confirmed release, not entering PLACE or failed placement', async () => {
  const send = await dashboard();
  send('/chat',books);
  send('/task_stage',{step:'GRASP'});
  send('/current_target_cube','red_cube_1');
  let out = send('/arm_status','grasp_succeeded');
  assert.equal(out.arm,2);
  out = send('/task_stage',{step:'PLACE'});
  assert.deepEqual([out.arm,out.placed],[3,0]);
  out = send('/arm_status','place_failed');
  assert.equal(out.placed,0);
  out = send('/arm_status','place_succeeded');
  assert.deepEqual([out.arm,out.placed],[4,1]);
  out = send('/arm_status','place_succeeded');
  assert.equal(out.placed,1);
  out = send('/task_stage',{step:'NAV_TO_BLOCK'});
  assert.equal(out.piece_index,2);
});

test('completion waits for confirmed releases and survives last-event ordering', async () => {
  const send = await dashboard();
  send('/chat',[{color:'blue',num:1,to:'B'}]);
  send('/task_stage',{step:'GRASP'});
  send('/current_target_cube','blue_cube_4');
  send('/task_stage',{step:'PLACE'});
  let out = send('/task_stage',{step:'WAIT_TASK'});
  assert.notEqual(out.phase,6);
  out = send('/arm_status','place_succeeded');
  assert.deepEqual([out.phase,out.piece_index,out.placed],[6,0,1]);
});

test('importing mid-run does not fabricate past progress', async () => {
  const send = await dashboard();
  send('/competition/status',{state:'started',tasks:books});
  let out = send('/competition/ready',{step:'NAV_TO_AREA',ready:false});
  assert.equal(out.phase,4);
  assert.equal(out.piece_index,-1);
  assert.equal(out.placed,-1);
  out = send('/arm_status','place_succeeded');
  assert.equal(out.placed,-1);
});

import type { Input } from "./types.ts";

type Task = { color: string; num: number; to: string };
type Output = {
  preparation: { problem: string; rule: string; preview_id: string; status: string };
  problem: string; rule: string; preview_id: string; notice: string;
  red_count: number; blue_count: number; red_areas: string; blue_areas: string;
  total: number; placed: number; piece_index: number; phase: number; arm: number;
};
type Event = Input<"/competition/problem"> | Input<"/competition/preview"> |
  Input<"/competition/status"> | Input<"/competition/ready"> | Input<"/chat"> |
  Input<"/task_stage"> | Input<"/arm_status"> | Input<"/current_target_cube">;

export const inputs = ["/competition/problem", "/competition/preview", "/competition/status",
  "/competition/ready", "/chat", "/task_stage", "/arm_status", "/current_target_cube"];
export const output = "/foxglove_script/competition_dashboard";

const empty = (): Output => ({ preparation: { problem: "", rule: "", preview_id: "", status: "" },
  problem: "", rule: "", preview_id: "", notice: "请粘贴当轮完整题目并解析",
  red_count: -1, blue_count: -1, red_areas: "", blue_areas: "", total: 0,
  placed: -1, piece_index: -1, phase: 0, arm: 0 });
let state = empty();
let active = false;
let observedStart = false;
let worked = false;
let cube = "";
let placedCubes: string[] = [];

function render(): Output {
  return { ...state, preparation: { problem: state.problem, rule: state.rule,
    preview_id: state.preview_id, status: state.notice } };
}

function parse(text: string): Record<string, unknown> | null {
  try { return JSON.parse(text) as Record<string, unknown>; } catch { return null; }
}
function tasksOf(value: unknown): Task[] | null {
  if (!Array.isArray(value) || value.length === 0) return null;
  const tasks: Task[] = [];
  for (const entry of value) {
    if (typeof entry !== "object" || entry === null) return null;
    const t = entry as Task;
    if (!["red", "blue"].includes(t.color) || !Number.isInteger(t.num) || t.num < 1 ||
        !["A", "B", "C"].includes(t.to)) return null;
    tasks.push(t);
  }
  return tasks;
}
function showTasks(tasks: Task[]): void {
  const reds = tasks.filter((t) => t.color === "red");
  const blues = tasks.filter((t) => t.color === "blue");
  state.red_count = reds.reduce((sum, t) => sum + t.num, 0);
  state.blue_count = blues.reduce((sum, t) => sum + t.num, 0);
  state.red_areas = Array.from(new Set(reds.map((t) => t.to))).sort().join("") || "-";
  state.blue_areas = Array.from(new Set(blues.map((t) => t.to))).sort().join("") || "-";
  state.total = state.red_count + state.blue_count;
}
function step(phase: string): void {
  const codes: Record<string, number> = { WAIT_TASK: 0, SELECT_BLOCK: 1,
    NAV_TO_BLOCK: 2, GRASP: 3, NAV_TO_AREA: 4, PLACE: 5 };
  if (phase !== "WAIT_TASK" && phase in codes) { active = true; worked = true; }
  if (phase === "WAIT_TASK" && worked) {
    const complete = observedStart && state.total > 0 && state.placed === state.total;
    state.phase = complete ? 6 : 7;
    state.piece_index = complete ? 0 : state.piece_index;
    state.notice = complete ? "全部放置完成" : "已停止或等待放置确认；请核对主控结果";
    active = false;
  } else if (!(state.phase === 6 && phase === "WAIT_TASK")) {
    state.phase = codes[phase] ?? state.phase;
  }
  if (phase === "GRASP") state.arm = 1;
  if (phase === "PLACE") state.arm = 3;
  if (active && observedStart) state.piece_index = Math.min(state.placed + 1, state.total);
}

export default function script(event: Event): Output {
  const raw = event.message.data;
  if (typeof raw !== "string") return render();
  if (event.topic === "/competition/problem") {
    const nested = parse(raw);
    if (!active) {
      state = empty();
      state.problem = typeof nested?.problem === "string" ? nested.problem : raw;
      state.rule = nested?.rule !== undefined ? `映射规则：${String(nested.rule)}` : "见题目末尾的映射规则";
      state.notice = "正在解析新题目";
      observedStart = false; worked = false;
    }
  } else if (event.topic === "/competition/preview") {
    const p = parse(raw);
    const tasks = tasksOf(p?.tasks);
    if (tasks !== null && !active) {
      showTasks(tasks);
      state.problem = typeof p?.problem === "string" ? p.problem : state.problem;
      state.rule = typeof p?.rule_text === "string" ? p.rule_text : state.rule;
      state.preview_id = typeof p?.preview_id === "string" ? p.preview_id : "";
      state.phase = 8; state.notice = "请核对数量和区域，再使用本次编号确认启动";
    } else if (typeof p?.error === "string") {
      state.notice = p.error;
      state.preview_id = "";
      if (!active) {
        state.red_count = -1; state.blue_count = -1;
        state.red_areas = ""; state.blue_areas = ""; state.phase = 9;
      }
    }
  } else if (event.topic === "/chat") {
    const tasks = tasksOf(parse(raw));
    if (tasks !== null) {
      showTasks(tasks);
      state.placed = 0; state.piece_index = 1; state.phase = 1; state.arm = 0;
      active = true; observedStart = true; worked = false;
      cube = ""; placedCubes = [];
      state.notice = "已收到实际执行任务";
    }
  } else if (event.topic === "/competition/status") {
    const p = parse(raw);
    if (p?.state === "waiting_problem") {
      state = empty(); active = false; observedStart = false; worked = false; placedCubes = [];
    } else if (p?.state === "started") {
      const tasks = tasksOf(p.tasks);
      if (tasks !== null) showTasks(tasks);
      state.notice = "已确认启动";
    } else if (p?.state === "start_rejected" || p?.state === "error") {
      state.notice = typeof p.error === "string" ? p.error : "题目或确认无效";
    }
  } else if (event.topic === "/competition/ready") {
    const p = parse(raw);
    if (typeof p?.step === "string" && (p.step !== "WAIT_TASK" || active || worked)) step(p.step);
  } else if (event.topic === "/task_stage") {
    const p = parse(raw);
    if (typeof p?.step === "string") step(p.step);
  } else if (event.topic === "/current_target_cube") {
    cube = raw;
  } else if (event.topic === "/arm_status") {
    if (raw === "grasp_succeeded") state.arm = 2;
    else if (raw === "place_succeeded") {
      state.arm = 4;
      if (observedStart && cube && !placedCubes.includes(cube)) {
        placedCubes.push(cube);
        state.placed = Math.min(state.total, state.placed + 1);
        if (state.phase === 7 && state.placed === state.total) step("WAIT_TASK");
      }
    } else if (raw === "grasp_failed") state.arm = 5;
    else if (raw === "place_failed" || raw === "place_lost") state.arm = 6;
    else if (raw.startsWith("place_reposition:")) state.arm = 7;
  }
  return render();
}

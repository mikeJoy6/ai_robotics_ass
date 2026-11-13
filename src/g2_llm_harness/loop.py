"""Agent 循环:问模型 → 执行它要的工具 → 把结果告诉它 → 再问,直到它不再调用工具。"""
import json

SYSTEM = """你是智元 G2 人形机器人的任务助手,只能通过提供的工具操作机器人。
用户要求手臂末端靠近某个物体时,根据物体的颜色选择 color,先 locate_object 再 approach_object;两次的 arm 要一致(用户说右手/右臂就都填 right),“最近的”指离这条手臂最近。
用户要求移动手臂末端(上抬、前伸、转手腕等)时用 move_arm_end;转头、转腰、转某个关节用 move_joints;回到录好的姿势用 go_to_pose。
用户说物体名(手机、杯子、笔……)要找或要看时,用 find_objects(英文名词)找,再用返回的 object_id;“看向/盯着某物”用 look_at_object,手臂去它上方用 approach_object(object_id=…),“最近的”选列表第一个。
不知道当前状态时先调用 get_robot_state。物体坐标都由工具计算,不要自己编造。
用户没有要求运动时,不要调用会让机器人运动的工具。
工具失败时,如实告诉用户失败原因,不要声称成功,也不要反复重试同一个失败的运动。
只有工具返回 executed 为 true,才能说机器人已经移动。"""


def run_agent(task, llm, gateway, tools, max_steps=8, trace=None):
    def log(kind, data):
        if trace:
            trace.write(json.dumps({"kind": kind, "data": data}, ensure_ascii=False) + "\n")
            trace.flush()

    messages = [{"role": "system", "content": SYSTEM}, {"role": "user", "content": task}]
    log("user", task)

    for _ in range(max_steps):
        reply = llm.chat(messages, tools)
        messages.append(reply)
        log("assistant", reply)

        calls = reply.get("tool_calls") or []
        if not calls:
            return reply.get("content", "")

        for call in calls:
            name = call["function"]["name"]
            print(f"  → 模型调用 {name} {call['function']['arguments']}", flush=True)
            result = _execute(call, gateway)
            print(f"  ← {json.dumps(result, ensure_ascii=False)}", flush=True)
            log("tool", {"name": name, "result": result})
            messages.append({"role": "tool", "tool_call_id": call["id"],
                             "content": json.dumps(result, ensure_ascii=False)})

    return f"超过最大步数 {max_steps},已停止。"


def _execute(call, gateway):
    fn = call["function"]
    try:
        args = json.loads(fn["arguments"] or "{}")
    except json.JSONDecodeError as e:
        return {"ok": False, "error": f"参数不是合法 JSON:{e}"}
    if not isinstance(args, dict):
        return {"ok": False, "error": "参数必须是 JSON 对象"}
    return gateway.execute(fn["name"], args)

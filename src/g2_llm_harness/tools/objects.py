"""给模型看的工具说明。工具名必须和 harness/gateway.py 里的一致。
物体相关的工具在这里,运动工具在 motion.py,make_tools 把两边合起来。"""
from .motion import make_motion_tools


def _tool(name, description, properties=None, required=()):
    return {"type": "function", "function": {
        "name": name, "description": description,
        "parameters": {"type": "object", "properties": properties or {}, "required": list(required)}}}


def make_tools(color_names, pose_names=()):
    return _object_tools(color_names) + make_motion_tools(pose_names)


def _object_tools(color_names):
    return [
        _tool("locate_object",
              "用头部 RGBD 相机按颜色定位物体,返回它在 base_link 下的坐标,并发布 TF target_object。"
              "同色物体有多个时,返回离 arm 那条手臂末端最近的那个。不会让机器人运动。",
              {"color": {"type": "string", "enum": list(color_names),
                         "description": "物体的颜色名称,例如青椒选 green"},
               "arm": {"type": "string", "enum": ["left", "right"],
                       "description": "接下来要用哪条手臂靠近它,不填就是左臂;"
                                      "必须和随后 approach_object 的 arm 一致"}},
              required=["color"]),
        _tool("get_left_end_pose",
              "读取左臂末端 arm_l_end_link 当前在 base_link 下的坐标。不会让机器人运动。"),
        _tool("approach_object",
              "让手臂末端移动到物体正上方,先规划直线,会撞就自动绕障。会让机器人运动。"
              "物体来自 find_objects 时填 object_id;来自 locate_object(按颜色)时不填 object_id。",
              {"object_id": {"type": "string",
                             "description": "find_objects 返回的编号,例如 phone_1;按颜色定位的物体不填"},
               "standoff_cm": {"type": "number",
                               "description": "末端停在物体上方多少厘米,可不填;只有用户明确要求时才填"},
               "arm": {"type": "string", "enum": ["left", "right"],
                       "description": "用哪条手臂,不填就是左臂;只有用户明确要求时才填 right"}}),
        _tool("find_objects",
              "用头部相机 + SAM 3 按物体名找画面里所有这种物体(任何物体都行,不限颜色)。"
              "返回每个物体的编号 object_id、base_link 坐标、离相机的距离、离画面中心的像素偏差,"
              "列表已按离相机由近到远排好,第一个最近。不会让机器人运动。",
              {"query": {"type": "string",
                         "description": "英文名词短语,例如 phone、cup、red block、pen。"
                                        "用户说中文时翻译成英文,只写物体名,不要写“最近的”之类的修饰"}},
              required=["query"]),
        _tool("look_at_object",
              "转头(只动头部 3 个关节,不动腰和手臂),让 find_objects 找到的某个物体位于画面中央。"
              "转完会再识别一次,返回物体离画面中心的像素偏差。会让机器人运动,需要用户确认。",
              {"object_id": {"type": "string", "description": "find_objects 返回的编号,例如 phone_1"}},
              required=["object_id"]),
    ]

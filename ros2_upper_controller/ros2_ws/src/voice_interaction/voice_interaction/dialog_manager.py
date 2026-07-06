from dataclasses import dataclass

from .llm_client import LLMClient


@dataclass(frozen=True)
class RouteResult:
    reply: str
    source: str


class DialogManager:
    def __init__(
        self,
        llm_client: LLMClient,
        system_prompt: str,
        enable_fast_commands: bool,
        enable_intent_routing: bool,
        max_rule_text_length: int,
    ):
        self.llm_client = llm_client
        self.system_prompt = system_prompt
        self.enable_fast_commands = enable_fast_commands
        self.enable_intent_routing = enable_intent_routing
        self.max_rule_text_length = max_rule_text_length
        self.fast_command_rules = [
            (('你好', '您好', '嗨'), '你好，我在。'),
            (('你是谁', '你叫什么', '介绍一下你自己'), '我是你的机器人语音助手。'),
            (('现在几点', '几点了', '当前时间'), '这个问题建议接系统时钟接口后再回答。'),
            (('再见', '拜拜', '结束对话'), '好的，有需要再叫我。'),
            (('谢谢', '感谢你', '辛苦了'), '不客气。'),
        ]
        self.intent_rules = [
            (('停止', '停下', '别动', '不要动'), '好的，我先停下。'),
            (('前进', '往前走', '向前走'), '好的，我开始前进。'),
            (('后退', '往后退', '向后退'), '好的，我开始后退。'),
            (('左转', '往左转', '向左转'), '好的，我向左转。'),
            (('右转', '往右转', '向右转'), '好的，我向右转。'),
            (('回充', '去充电', '回去充电'), '好的，我现在回充。'),
        ]

    def reply(self, user_text: str) -> RouteResult:
        text = self._normalize(user_text)
        if not text:
            return RouteResult(reply='你刚才没有说话。', source='empty')

        if self.enable_fast_commands:
            fast_reply = self._match_rules(text, self.fast_command_rules)
            if fast_reply:
                return RouteResult(reply=fast_reply, source='fast_command')

        if self.enable_intent_routing and len(text) <= self.max_rule_text_length:
            intent_reply = self._match_rules(text, self.intent_rules)
            if intent_reply:
                return RouteResult(reply=intent_reply, source='intent_rule')

        return RouteResult(
            reply=self.llm_client.generate(self.system_prompt, text),
            source='llm',
        )

    def _normalize(self, user_text: str) -> str:
        return ''.join(user_text.strip().split())

    def _match_rules(self, text: str, rules: list[tuple[tuple[str, ...], str]]) -> str:
        for keywords, reply in rules:
            if any(keyword in text for keyword in keywords):
                return reply
        return ''

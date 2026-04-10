# High-level mission command translated from NL / dispatched to mission_manager
string command_type     # e.g., "navigate_to", "patrol", "explore", "stop", "return_home"
string parameters_json  # JSON-encoded command parameters
string natural_language # Original NL text (for logging / audit)
uint8 priority          # 0=low, 1=normal, 2=high, 3=emergency
string mission_id       # UUID or caller-supplied ID

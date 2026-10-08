from dvrk_simulator_base.command_mailbox import CommandMailboxes


def test_newest_arm_motion_wins_across_servo_and_move():
    mailboxes = CommandMailboxes(discrete_capacity=2)
    first = mailboxes.submit_servo("servo_jp", "old")
    move = mailboxes.submit_discrete("move_jp", "move")
    newest = mailboxes.submit_servo("servo_jp", "new")

    assert first.sequence < move.sequence < newest.sequence
    assert [(item.channel, item.payload) for item in mailboxes.drain()] == [
        ("servo_jp", "new"),
    ]
    assert mailboxes.counters.received == 3
    assert mailboxes.counters.coalesced == 2


def test_discrete_queue_is_bounded():
    mailboxes = CommandMailboxes(discrete_capacity=1)
    assert mailboxes.submit_discrete("state_command", "enable") is not None
    assert mailboxes.submit_discrete("state_command", "disable") is None
    assert mailboxes.submit_discrete("move_jp", 1) is not None
    assert [(item.channel, item.payload) for item in mailboxes.drain()] == [
        ("state_command", "enable"),
        ("move_jp", 1),
    ]
    assert mailboxes.counters.rejected == 1

"""Simulation fetch workflow, independent of ROS messages and action transport.

Ports own navigation, perception, joint execution and grasp evidence. The fixed
joint poses below are test fixtures, not a real grasp policy or collision plan.
"""


async def fetch_drink(port, context, object_id, strategy):
    port.stage(context, 'NAVIGATE')
    await port.navigate(context, 'pickup')
    port.stage(context, 'LOCATE')
    await port.locate(context, object_id)
    port.stage(context, 'APPROACH')
    await port.joints(context, strategy, 'arm', tuple(.25 for _ in port.profile.groups['arm']))
    port.stage(context, 'GRASP')
    target = tuple(.45 for _ in port.profile.groups['hand'])
    await port.joints(context, strategy, 'hand', target)
    port.confirm_grasp(object_id, target)
    port.stage(context, 'RETURN')
    await port.navigate(context, 'home')

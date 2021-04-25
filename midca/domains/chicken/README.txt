Why did the chicken cross the road? Clucky has the goal onleft(clucky)
and executes the plan cross-left(clucky) to be with lady-clucky
because he loves her and she loves him. See Quick Start in the MIDCA
V1.5 Manual at midca/docs/MIDCA_Doc_9_8.docx.

Now, if we used a goal monitor with this example, we would monitor the
conjunct (and (loves clucky lady-clucky) (loves lady-clucky clucky)),
the reason clucky has the goal onleft(clucky). If during execution
clucky finds out that lady-clucky no longer loves him, the monitor
should fire during the Interpret phase, drop the goal, and not execute
the plan. clucky stays on the right hand side of the road (a marital
separation of sorts).

Can you, the new user of MIDCA, implement a simple goal monitor to
recreate this situation? See Dannenhauer & Cox, 2018.

Dannenhauer, Z. A., & Cox, M. T. (2018). Rationale-based perceptual
monitors. AI Communications, 31 (2), 197-212.
https://onedrive.live.com/?authkey=%21APF92MPxKVFvhLU&cid=FDD01A401DA67D10&id=FDD01A401DA67D10%214038&parId=FDD01A401DA67D10%21105&o=OneUp

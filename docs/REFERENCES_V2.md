# v2 implementation references

The uploaded project is the implementation baseline; its exact ZIP hash and original module hashes are recorded in SOURCE_BASE.json. No public repository was substituted for the uploaded bytes.

1. Steven M. LaValle, Planning Algorithms, §15.3.1 Dubins Curves. Six candidate words and the forward, bounded-curvature formulation. <https://lavalle.pl/planning/node821.html>
2. SciPy official documentation, scipy.optimize.minimize(method="SLSQP"). Constrained numerical optimizer API used for local Bezier handle and primitive-time parameters. <https://docs.scipy.org/doc/scipy/reference/optimize.minimize-slsqp.html>
3. OpenAI official AGENTS.md guide. Used only for the follow-on development contract file, not a runtime dependency. <https://learn.chatgpt.com/docs/agent-configuration/agents-md>

The code's derivative equations, conservative whole-curve enclosures, test results and limitation statements are specific to this implementation. The above references do not certify this project, endorse a manufacturer model or prove global obstacle-aware/time optimality. Scientific Python dependencies retain their own licenses; they are installed externally rather than vendored.

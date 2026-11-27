# Code Review

Rover and arm developers review each other's code. The mentor does not review every PR.

Branch naming and the git workflow are in [CONTRIBUTING](https://github.com/UCUSpaceRobotics/.github/blob/main/.github/CONTRIBUTING.md).


## Who reviews

Owners of each folder are set in [`.github/CODEOWNERS`](../../.github/CODEOWNERS) as teams:

| Area | Folders | Owner team |
|------|---------|------------|
| Rover | `src/rover/`, `scripts/navigation/` | `rover-software` |
| Arm and panel | `src/arm/`, `src/panel/`, `scripts/arm/`, `docs/arm/`, `docs/panel/` | `manipulator-software` |
| Shared | `src/indomitus/` (interfaces, CAN), `docker/`, `.github/`, `system/`, other `scripts/`, everything else | `maintainers` (tech leads) |

When a PR touches a team's folders, GitHub **assigns 2 people from that team** as reviewers (team auto-assignment, load balance: whoever has the fewest open reviews). The author is never assigned to their own PR. Only the assigned people are notified, not the whole team.

* A PR needs **2 approvals**, and at least one of them from the owner team of every area it touches. A PR that changes `src/rover/` and `src/indomitus/` needs a rover developer and a tech lead among its approvers.
* **The assigned reviewers are responsible for the review.** Both of them review, not only the first one to answer. Ping them in the team chat if the PR waits.
* **Nobody merges their own PR without a review**, tech leads included.
* If an assigned reviewer cannot review within 2 working days (absent, overloaded), they say so in the PR and the **tech lead of the team reviews instead**. Someone who is away for a longer time is excluded from auto-assignment in the team settings until they are back.


## Before opening a PR

* Keep it small: **up to ~400 changed lines**. Split bigger work into several PRs (e.g. interface first, then the node, then the launch files).
* Fill in the PR template: what changed, how it was tested, what can break.
* CI must pass (build and `colcon test`).
* Open a draft PR early if you want feedback on the approach.


## Reviewing

* Review within **2 working days**. If you cannot, say so in the PR (see above: the tech lead takes over).
* Check: does it do what the PR says, how was it tested, what happens on failure (lost CAN message, missing topic, wrong parameter), is it readable for the next person.
* Use **Request changes** only for things that must be fixed. Mark optional remarks as `nit:`.
* The author resolves the threads and asks for a re-review after pushing fixes.


## Mentor

The mentor is asked for a review only when it is needed. Add the `needs-mentor` label to PRs that:

* change interfaces between the rover and the arm (`src/indomitus/`, CAN protocol, shared messages);
* touch motor control limits, power or emergency stop;
* make an architectural decision (new package, new node layout, new dependency).

The mentor also looks through a few merged PRs each sprint to check the quality of reviews.


## GitHub setup

Done once by an org admin; listed here so it can be checked or restored.

* Teams `rover-software` (child of `rover`) and `manipulator-software` (child of `manipulator`) contain only the software developers of each team; tech leads are members of their team and of `maintainers`.
* `rover-software`, `manipulator-software` and `maintainers` have **Write** access to this repository (without it GitHub ignores them in `CODEOWNERS`).
* Each of the three teams: *Settings → Code review → Auto assignment* on, **2 reviewers**, routing **Load balance**, *Don't notify the entire team* on.
* Branch protection on `develop`: require a pull request, **2 approvals**, *Require review from Code Owners*, required CI status checks.
* Label `needs-mentor` exists in the repository.

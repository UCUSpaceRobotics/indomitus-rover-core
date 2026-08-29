#!/usr/bin/env python3
"""Async service calls that cannot pile up on each other.

A button held down, a switch bounced, or a service that has gone quiet will all
otherwise queue request after request, and the replies then land in an order
nobody controls. One in flight at a time is the whole policy.
"""


class GuardedCall:
    """Single-in-flight wrapper around a service client.

    Drops a request rather than queueing it when one is already outstanding or
    the service is not up yet, and says so by returning False, so the caller
    can log the refusal instead of silently doing nothing.
    """

    def __init__(self, client):
        self._client = client
        self.pending = False

    def call(self, request, on_done) -> bool:
        if self.pending or not self._client.service_is_ready():
            return False

        self.pending = True

        def _wrapped(future):
            self.pending = False
            on_done(future)

        self._client.call_async(request).add_done_callback(_wrapped)
        return True

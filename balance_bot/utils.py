from typing import Callable, Any, Optional
import time

def find_threshold(
    name: str,
    initial: float,
    min_val: float,
    max_val: float,
    action_fn: Callable[[float], Any],
    check_fn: Callable[[Any], bool],
    heartbeat_fn: Optional[Callable[[], None]] = None,
    epsilon: float = 1.0  # Precision for float search
) -> Optional[float]:
    """
    Finds the minimum value in [min_val, max_val] where check_fn(action_fn(value)) is True.
    Uses binary search for efficiency.
    """

    print(f"[{name}] Starting threshold search (Range: {min_val}-{max_val}, Start: {initial})")

    # Helper for binary search
    def binary_search(low, high):
        best = None

        while high - low > epsilon:
            if heartbeat_fn:
                heartbeat_fn()

            mid = (low + high) / 2.0

            print(f"[{name}] Checking {mid:.2f}...")
            result = action_fn(mid)
            if check_fn(result):
                best = mid
                high = mid # Try smaller
            else:
                low = mid # Need larger

        return best

    # Check initial guess first
    if heartbeat_fn:
        heartbeat_fn()

    print(f"[{name}] Checking initial guess {initial:.2f}...")
    res_initial = action_fn(initial)
    initial_passed = check_fn(res_initial)

    if initial_passed:
        print(f"[{name}] Initial value {initial} passed. Searching downwards [ {min_val} .. {initial} ].")
        # We know initial works. Search [min, initial]
        # But we need to be careful:
        # binary_search above returns 'best', which is the smallest working value found.
        # If the range is [min, initial], and min doesn't work, it finds the boundary.
        # If min works, it returns min (approx).

        # We need to verify if we can go lower than initial.
        found = binary_search(min_val, initial)
        if found is not None:
            print(f"[{name}] Found lower threshold at {found:.2f}")
            return found
        else:
            # If binary search found nothing (implying min..initial all failed? Impossible since initial passed),
            # Wait, my binary search implementation:
            # if check(mid) passes, best=mid, high=mid.
            # So it will eventually converge to something <= initial.
            # Unless [min, initial] is so small?
            return initial

    else:
        print(f"[{name}] Initial value {initial} failed. Searching upwards [ {initial} .. {max_val} ].")
        # We know initial failed. Search [initial, max]
        found = binary_search(initial, max_val)

        if found is not None:
            print(f"[{name}] Found threshold at {found:.2f}")
            return found
        else:
            print(f"[{name}] Failed to find threshold up to {max_val}.")
            return None

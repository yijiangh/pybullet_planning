from itertools import cycle, islice

def implies(p1, p2):
    return not p1 or p2

def roundrobin(*iterables):
    """roundrobin('ABC', 'D', 'EF') --> A D E B F C

    https://docs.python.org/3.1/library/itertools.html#recipes
    Recipe credited to George Sakkis
    """
    pending = len(iterables)
    nexts = cycle(iter(it).__next__ for it in iterables)
    while pending:
        try:
            for next in nexts:
                yield next()
        except StopIteration:
            pending -= 1
            nexts = cycle(islice(nexts, pending))

def chunks(sequence, n=1):
    """get [i, i+n] sublist of a given sequence
    """
    for i in range(0, len(sequence), n):
        yield sequence[i:i + n]

def safe_zip(sequence1, sequence2):
    """zip with safeguarding on the length
    """
    assert len(sequence1) == len(sequence2)
    return zip(sequence1, sequence2)

def get_pairs(sequence):
    """get a sequece of (seq[i], seq[i+1]), i=0~n-1
    """
    return list(safe_zip(sequence[:-1], sequence[1:]))

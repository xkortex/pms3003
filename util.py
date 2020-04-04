import os
import errno
import datetime



class LitePstruct(dict):
    def __getattr__(self, name):
        if name in self.__slots__:
            return self[name]
        return self.__getattribute__(name)


class TxnObj(LitePstruct):
    __slots__ = ['trigger', 'source', 'dest']

    def __init__(self, trigger, source, dest):
        super(TxnObj, self).__init__(trigger=trigger, source=source, dest=dest)


def pathsafe_timestamp(now=None, show_micros=False, show_millis=False):
    # type: (Optional[datetime.datetime], Optional[bool]) -> str
    """
    Filesystem name safe timestamp string.
    :param now: datetime object of time to process
    :param show_micros: - Format with microseconds
    :return:
    """
    now = now or datetime.datetime.now()
    show_micros = show_millis or show_micros
    fmt = "%Y%m%d_%H%M%S{}".format('.%f' * show_micros)
    s = now.strftime(fmt)
    if show_millis:
        s = s[:-3]
    return s


def make_path(path, from_file=False, verbose=False):
    """
    Make a path, ignoring already-exists error. Python 2/3 compliant.
    Catch any errors generated, and skip it if it's EEXIST.
    :param path: Path to create
    :type path: str, pathlib.Path
    :param from_file: if true, treat path as a file path and create the basedir
    :return: Path created or exists
    """
    path = str(path)  # coerce pathlib.Path
    if path == '':
        raise ValueError("Path is empty string, cannot make dir.")

    if from_file:
        path = os.path.dirname(path)
    try:
        os.makedirs(path)
        if verbose:
            print('Created path: {}'.format(path))
    except OSError as exception:
        if exception.errno != errno.EEXIST:
            raise
        if verbose:
            print('Tried to create path, but exists: {}'.format(path))

    return path
#!/usr/bin/env python
import inspect
from time import strftime

class levels:
    OK = True
    WARNING = 10
    DEBUG = 30
    ERROR = False


class colors:
    OK = '\033[92m'
    INFO = ''
    DEBUG = '\033[95m'
    WARNING = '\033[93m'
    ERROR = '\033[91m'
    GROUPTEST = '\033[94m'
    ENDC = '\033[0m'

def time():
    return strftime('%H:%M:%S')

def out(message):
    print time(), message

def ok(message):
    out(colors.OK +      '[OK]      ' + colors.ENDC + message)

def info(message):
    out(colors.INFO +    '[INFO]    ' + colors.ENDC + message)

def debug(message):
    out(colors.DEBUG +   '[DEBUG]   ' + colors.ENDC + message)

def warning(message):
    out(colors.WARNING + '[WARNING] ' + colors.ENDC + message)

def error(message):
    out(colors.ERROR +   '[ERROR]   ' + colors.ENDC + message)

def test(function, *args, **kwargs):
    result = function(*args, **kwargs)
    message = function.__name__
    message += ' ' + str(args)
    message = message.replace(',)', ')')

    (frame, _, _, caller, _, _) = inspect.stack()[1]

    if caller == '<module>':
        caller = inspect.getmodule(frame).__name__

    message = message + caller.rjust(80 - 19 - len(message))

    # switch
    { levels.OK: ok,
      levels.WARNING: warning,
      levels.DEBUG: debug,
      levels.ERROR: error,
      None: info }[result](message)
    return result

def add_test(l, f, *args, **kwargs):
    l.append( { 'function': f, 'args': args, 'kwargs': kwargs } )

def group_test(l):
    (frame, _, _, caller, _, _) = inspect.stack()[1]
    if caller == '<module>':
        caller = inspect.getmodule(frame).__name__

    c = 0

    for e in l:
        result = e['function']( *e['args'], **e['kwargs'])
        c += bool(result)
        message = e['function'].__name__
        message += ' ' + str(e['args'])
        message = message.replace(',)', ')')

        message = message + colors.GROUPTEST + caller.rjust(80 - 19 - len(message)) + colors.ENDC

        # pythonic way of doing a switch statement
        { levels.OK: ok,
          levels.WARNING: warning,
          levels.DEBUG: debug,
          levels.ERROR: error,
          None: info }[result](message)

    if c == len(l):
        ok(colors.GROUPTEST + caller + colors.ENDC + colors.OK + ' [%d/%d]'%(c,len(l)) + colors.ENDC)
    else:
        error(colors.GROUPTEST + caller + colors.ENDC + colors.ERROR + ' [%d/%d]'%(c,len(l)) + colors.ENDC)

            


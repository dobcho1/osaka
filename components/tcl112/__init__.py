from .climate import Tcl112Climate

def to_code(config):
    yield Tcl112Climate.new(config)

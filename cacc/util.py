import importlib

def cls_from_str(clsname):
    module_name, class_name = clsname.rsplit(".", 1)
    module = importlib.import_module(module_name, package='cacc')
    cls = getattr(module, class_name)
    return cls

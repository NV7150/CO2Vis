import pyqrcode


def create_code(s, file_name):
    code = pyqrcode.create(s, error='L', version=3, mode='binary')
    code.png(file_name, scale=5, module_color=[0, 0, 0, 128], background=[255, 255, 255])


create_code('2164983504', 'resources/ID1.png')
create_code('2164987842', "resources/ID2.png")

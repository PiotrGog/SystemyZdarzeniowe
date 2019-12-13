from src import consts

print(consts.RobotStatus)


def foo(text):
    print(text)


dict_test = {"foo": foo}

dict_test['foo']("jakis tekst")

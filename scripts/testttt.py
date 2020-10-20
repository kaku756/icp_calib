


def callback():
    a=b
    print a
def func1():
    b=12
    callback()

if __name__ == '__main__':
    b=12
    func1()

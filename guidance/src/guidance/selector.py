import zenoh
from zenoh import Session


class FSMSelector:
    def __init__(self, session: Session):
        self.session = session

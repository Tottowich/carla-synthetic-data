
class ProcessResult:
    def __init__(self, success:bool, message:str=None):
        self.success = success
        self.status = "Success" if success else "Failure"
        self.message = message
    def __bool__(self):
        return self.success
    @property
    def get_message(self):
        return self.message

    def __str__(self):
        message = f"Message: {self.message}" if self.message else ""
        return f"{self.status} - {message}"

import sqlite3 


class FollowMeCtrl30(object):
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2017-07-08 20:45:02 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 3654
        self.input_vars = ['env1']
        self.database = sqlite3.connect("FollowMeCtrl30_db.db")
        self.database.row_factory = sqlite3.Row
        self.cursor = self.database.cursor()

    def move(self, env1):
        """Given inputs, take move and return outputs.

        @rtype: dict
        @return: dictionary with keys of the output variable names:
            ['loc']
        """
        output = dict()
        req_arg = dict()
        req_arg["current_state"] = self.state
        req_arg["env1"]=int(env1)
        self.cursor.execute("select next_state,loc from state_trans where current_state=:current_state and env1=:env1",req_arg)
        res = self.cursor.fetchone()
        if res is not None:
            (self.state,output["loc"])=(res["next_state"],res["loc"])
        else:
            self.cursor.execute("select is_dead_end_state from state_trans where current_state="+str(self.state))
            res_else = self.cursor.fetchone()
            if res_else is not None and res_else["is_dead_end_state"] == 1:
                raise Exception("Reached dead-end state !")
            elif res_else is not None:
                self._error(env1)
            else:
                raise Exception("Unrecognized state : state = "+str(self.state))
        return output

    def _error(self, env1):
        raise ValueError("Unrecognized input: " + (
            "env1 = {env1}; ").format(
                env1=env1))

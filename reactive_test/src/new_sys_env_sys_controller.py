import sqlite3 


class new_sys_env_sysController(object):
    """Mealy transducer.

    Internal states are integers, the current state
    is stored in the attribute "state".
    To take a transition, call method "move".

    The names of input variables are stored in the
    attribute "input_vars".

    Automatically generated by tulip.dumpsmach on 2017-08-22 11:36:31 UTC
    To learn more about TuLiP, visit http://tulip-control.org
    """
    def __init__(self):
        self.state = 8414
        self.input_vars = ['env1', 'env0', 'target_Quad9']
        self.database = sqlite3.connect("new_sys_env_sysController_db.db")
        self.database.row_factory = sqlite3.Row
        self.cursor = self.database.cursor()

    def move(self, env1, env0, target_Quad9):
        """Given inputs, take move and return outputs.

        @rtype: dict
        @return: dictionary with keys of the output variable names:
            ['follow_complete', 'Quad9', 'stage_Quad9']
        """
        output = dict()
        req_arg = dict()
        req_arg["current_state"] = self.state
        req_arg["env1"]=int(env1)
        req_arg["env0"]=int(env0)
        req_arg["target_Quad9"]=int(target_Quad9)
        self.cursor.execute("select next_state,follow_complete,Quad9,stage_Quad9 from state_trans where current_state=:current_state and env1=:env1 and env0=:env0 and target_Quad9=:target_Quad9",req_arg)
        res = self.cursor.fetchone()
        if res is not None:
            (self.state,output["follow_complete"],output["Quad9"],output["stage_Quad9"])=(res["next_state"],res["follow_complete"],res["Quad9"],res["stage_Quad9"])
        else:
            self.cursor.execute("select is_dead_end_state from state_trans where current_state="+str(self.state))
            res_else = self.cursor.fetchone()
            if res_else is not None and res_else["is_dead_end_state"] == 1:
                raise Exception("Reached dead-end state !")
            elif res_else is not None:
                self._error(env1, env0, target_Quad9)
            else:
                raise Exception("Unrecognized state : state = "+str(self.state))
        return output

    def _error(self, env1, env0, target_Quad9):
        raise ValueError("Unrecognized input: " + (
            "env1 = {env1}; "
            "env0 = {env0}; "
            "target_Quad9 = {target_Quad9}; ").format(
                env1=env1,
                env0=env0,
                target_Quad9=target_Quad9))

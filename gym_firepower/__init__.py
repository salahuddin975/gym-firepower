from gym.envs.registration import register

register(
    id='firepower-v0',
    entry_point='gym_firepower.envs:FirePowerEnv',
    kwargs={
            "geo_file" : None, 
            "network_file" : None, 
            "scaling_factor" :   6,
            "non_convergence_penalty" :  None, 
            "protection_action_penalty" :    None,
            "seed" : 34},
)

from G500Env import G500Env

env = G500Env()

terminated = False
truncated = False

total_reward = 0

while (env.step_counter < 2000):

    #env.enviar_command_proves()
    env.step(action=1)           
    print("\n Step: ", env.step_counter)
    reward = env.calculate_reward()
    print("Reward: ", reward)

    env.step_counter += 1
        
                   
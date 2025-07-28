from G500Env import G500Env

env = G500Env()

obs, info = env.reset(seed=123, options={"low": -7.2, "high": 7.2})

terminated = False
truncated = False

total_reward = 0


while True:
    while not (terminated or truncated):

        obs, reward, terminated, truncated, info = env.step(action=1)   
        
        print("\n Step: ", env.step_counter)
        print("Observation:", obs)
        print("Reward:", reward)
        print("Terminated:", terminated, "| Truncated:", truncated)

        if terminated:
            print("OBJECTIU ASSOLIT :) Resetejant entorn...\n")
            observation, info = env.reset()
            

        if truncated:
            print("TRUNCATED: S'han superat els 20 segs :(")
            observation, info = env.reset()

        terminated = False
        truncated = False
                        
from openai import OpenAI

class GPTagent():
    def __init__(self, name):
        self.api_key = your_key
        self.client = OpenAI(api_key=self.api_key)
        self.messages = []  
        self.response = ""


        with open(prompt, 'r') as file:
            planner_prompt = file.read()

        with open(prompt, 'r') as file:
            coder_prompt = file.read()    

        if name == "planner":
           self.model = 'gpt-4'
           self.initial_prompts(planner_prompt)
        elif name == 'coder':
            self.initial_prompts(coder_prompt)
            self.model = 'gpt-4'


    def get_response(self, command):
        self.messages.append({
            "role": "user",
            "content": command
        })

        chat_completion = self.client.chat.completions.create(
            messages=self.messages,
            model=self.model,
            temperature=0.0,
            max_tokens = 3000
        )

        assistant_response = chat_completion.choices[0].message.content

        self.messages.append({
            "role": "assistant",
            "content": assistant_response
        })

        self.response = assistant_response

        return assistant_response
    
    def initial_prompts(self, prompts):
        self.messages.append({
            "role": "assistant",
            "content": prompts
        })



# if __name__ == "__main__":

#     planner = GPTagent('planner')
#     res = planner.get_response("I want to grasp the root of the appendix")
#     print("planner_1:",res)


#     coder = GPTagent('coder')
#     res1 = coder.get_response(res)
#     print("coder:",res1)
    

#     planner = GPTagent('planner')
#     res = planner.get_response("i want to grasp head using left psm")
#     print("planner_1:",res)

#     a = planner.get_response("The Observation from the scene is shown as follows :\
#                  Now the left grripper position is: [0, 0, 0.2, 0, 0, 0, 1] and its jaw is closed\
#                  the right grripper position is: [0, 0.8, 0.4, 0, 0, 0, 1] and its jaw is open\
# ")
#     print("planner_2:",a)


#     q = planner.get_response("use right psm to cut the root")
#     print("planner_3:",q)

#     q1 = planner.get_response("now move left psm to [1,0,0,1,0,0,0], and remember this position")
#     print("planner_4:",q1)

#     coder = GPTagent('coder')
#     res1 = coder.get_response(res)
#     print("coder:",res1)

    




import tkinter as tk
from tkinter import scrolledtext, font
Your_path
from agent.gpt_agent import GPTagent
import subprocess
import io
import sys

class GUI():
    def __init__(self):
        self.app = tk.Tk()
        
        self.planner = GPTagent('planner')
        self.coder = GPTagent('coder')

        self.planner_response = ''
        self.coder_response = ''

        self.font = font.Font(family="Helvetica", size=14)

        self.prompt_label = tk.Label(self.app, font=self.font ,text="Commands:")
        self.prompt_entry = tk.Text(self.app, height=5, width=100)

        self.button_block = tk.Frame(self.app)
        self.submit_button = tk.Button(self.button_block, text="Send", font=self.font)
        self.action_button = tk.Button(self.button_block, text="Action", font=self.font)
        self.paused_button = tk.Button(self.button_block, text="Pause", font=self.font)
        self.continue_button = tk.Button(self.button_block, text="Continue", font=self.font)

        self.response_label = tk.Label(self.app, text="Response:",font=self.font)
        self.response_display = scrolledtext.ScrolledText(self.app, height=20, width=100, state=tk.DISABLED)

        self.create_GUI()
         
    def create_GUI(self):
        self.app.title("Agent")
        self.app.geometry("600x400")

        self.coder_response = ""
        
        self.prompt_label.pack(pady=10)
        self.prompt_entry.pack(pady=10)

        self.button_block.pack(pady=10)
        self.submit_button.grid(row=0, column=0, padx=0)
        self.action_button.grid(row=0, column=1, padx=5)
        self.paused_button.grid(row=0, column=2, padx=10)
        self.continue_button.grid(row=0, column=3, padx=15)

        self.response_label.pack(pady=10)
        self.response_display.pack(pady=10)

    def start_peg_transfer(self):
        print("peg_transfer!")
        # pass

    def get_response(self):
        command = self.prompt_entry.get("1.0", tk.END).strip()
        if command:
            self.planner_response = self.planner.get_response(command)
            self.display_response(self.planner_response)
            self.prompt_entry.delete("1.0", tk.END)
        else:
            self.display_response("Please enter a command.")

    def get_function(self, plan):
        if plan:
            self.coder_response = self.coder.get_response(plan)
        else:
            self.display_response("Please provide a plan.") 

        return self.coder_response      

    def display_response(self, response):
        self.response_display.config(state=tk.NORMAL)
        self.response_display.delete("1.0", tk.END)
        self.response_display.insert(tk.END, response)
        self.response_display.config(state=tk.DISABLED)
    

    def run(self):
        self.app.mainloop()

if __name__ == "__main__":
    app = GUI()
    app.run()

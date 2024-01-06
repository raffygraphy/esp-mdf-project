import keyboard

def alt_tab_remap(e):
    if e.event_type == keyboard.KEY_DOWN and e.name == 'alt':
        keyboard.press('caps lock')
        keyboard.press('tab')
    elif e.event_type == keyboard.KEY_UP and e.name == 'alt':
        keyboard.release('tab')
        keyboard.release('caps lock')

keyboard.hook_key('alt', alt_tab_remap)

try:
    keyboard.wait('esc')  # Wait for the 'esc' key to be pressed to exit the script
except KeyboardInterrupt:
    pass
finally:
    keyboard.unhook_all()  # Unhook all the keyboard events when the script exits

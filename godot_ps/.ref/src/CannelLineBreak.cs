using Godot;
using System;

public partial class CannelLineBreak : TextEdit
{
    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
    }

    public override void _GuiInput(InputEvent @event)
    {
        if (@event is InputEventKey keyEvent && keyEvent.Pressed && keyEvent.Keycode == Key.Enter)
        {
            //@event.Handled = true; // デフォルト動作を抑制
            Text = ""; // テキストをクリア
            SetCaretLine(0); // カレットの行位置を 1 行目に設定
            SetCaretColumn(0); // カレットの列位置を 0 に設定
            QueueRedraw(); // 視覚的な更新を強制
        }
    }
}

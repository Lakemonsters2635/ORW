// DisplayFrame

void DisplayFrame(rs2::video_frame& frame, const char* Title)
{
	ImGui::Begin(Title);
	{
		auto width = frame.get_width();
		auto height = frame.get_height();

		std::string child(Title);
		child.append(" Stream");

		texture frame_texture;
		frame_texture.render(frame, { 0, 0, 200.0f * width / height, 200 });

		// Using a Child allow to fill all the space of the window.
		// It also alows customization
		ImGui::BeginChild(child.c_str());
		// Get the size of the child (i.e. the whole draw size of the windows).
		ImVec2 wsize = ImGui::GetWindowSize();
		ImGui::Image((ImTextureID)frame_texture.get_gl_handle(), wsize);
		ImGui::EndChild();
	}
	ImGui::End();
}

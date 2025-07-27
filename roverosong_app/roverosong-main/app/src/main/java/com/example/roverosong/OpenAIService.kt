import com.example.roverosong.OpenAIRequest
import com.example.roverosong.OpenAIResponse
import retrofit2.Call
import retrofit2.http.Body
import retrofit2.http.Headers
import retrofit2.http.POST
import retrofit2.http.Header

interface OpenAIService {
    @POST("v1/chat/completions")
    fun getChatCompletion(
        @Header("Authorization") auth: String,  //API 키를 동적으로 전달
        @Body request: OpenAIRequest
    ): Call<OpenAIResponse>
}
